#include "gps_voo.h"
#include <Wire.h>
#include <TinyGPSPlus.h>

// ---------------------------------------------------------------------------
// Configuração
// ---------------------------------------------------------------------------
namespace {

// I2C (u-blox DDC)
constexpr uint8_t  ENDERECO_GPS_I2C                = 0x42;
constexpr int      PINO_SDA_I2C                    = 21;
constexpr int      PINO_SCL_I2C                    = 22;
constexpr uint32_t CLOCK_I2C_HZ                    = 400000;
constexpr uint8_t  REGISTRADOR_BYTES_DISPONIVEIS   = 0xFD;  // 0xFD/0xFE = contador
constexpr uint8_t  REGISTRADOR_FLUXO_DADOS         = 0xFF;
constexpr uint8_t  TAMANHO_BLOCO_I2C_BYTES         = 32;
constexpr uint16_t LIMITE_BYTES_POR_LEITURA        = 256;   // evita ficar preso lendo

// Configuracao do modulo (UBX). Nao e gravada na flash: reaplicada a cada boot.
constexpr bool     CONFIGURAR_MODULO_NO_INICIO     = true;
constexpr uint8_t  TAXA_ATUALIZACAO_HZ             = 5;     // NEO-6M ate 5, M8 ate 10
constexpr uint8_t  MODELO_DINAMICO_AEREO_4G        = 8;     // "Airborne <4g"
constexpr uint8_t  TAMANHO_MAXIMO_CARGA_UBX_BYTES  = 40;

// Logica de voo
constexpr double   LIMIAR_VELOCIDADE_VERTICAL_MS   = 5.0;   // classifica subida/descida
constexpr double   PASSO_MARCO_M                   = 100.0;
constexpr double   FATOR_FILTRO_REFERENCIA         = 0.2;   // media movel na rampa
constexpr double   QUEDA_APOGEU_M                  = 3.0;   // queda abaixo da maxima
constexpr uint8_t  LEITURAS_CONFIRMA_APOGEU        = 3;
constexpr double   ALTITUDE_MINIMA_APOGEU_M        = 15.0;  // ignora ruido no solo
constexpr unsigned long TIMEOUT_DADOS_MS           = 3000;

constexpr bool     DEBUG_GPS                       = true;

// ---------------------------------------------------------------------------
// Estado interno do modulo
// ---------------------------------------------------------------------------
TinyGPSPlus gps;

bool   referencia_definida   = false;
double altitude_inicial_m    = 0.0;
double altitude_absoluta_m   = 0.0;
double altitude_anterior_m   = 0.0;   // absoluta, usada para a velocidade
double altitude_relativa_m   = 0.0;
double altitude_maxima_m     = 0.0;
double velocidade_vertical_ms = 0.0;
double proximo_marco_m       = PASSO_MARCO_M;

VariacaoVertical variacao_vertical = VariacaoVertical::ESTAVEL;

bool          primeira_leitura       = true;
bool          apogeu_detectado       = false;
uint8_t       contagem_queda         = 0;
unsigned long instante_anterior_ms   = 0;
unsigned long ultima_leitura_ms      = 0;

// ---------------------------------------------------------------------------
// I2C: leitura do fluxo NMEA
// ---------------------------------------------------------------------------
uint16_t ler_bytes_disponiveis() {
  Wire.beginTransmission(ENDERECO_GPS_I2C);
  Wire.write(REGISTRADOR_BYTES_DISPONIVEIS);
  if (Wire.endTransmission(false) != 0) return 0;

  if (Wire.requestFrom(ENDERECO_GPS_I2C, (uint8_t)2) != 2) return 0;
  const uint16_t alto  = Wire.read();
  const uint16_t baixo = Wire.read();
  return (alto << 8) | baixo;
}

uint8_t ler_bloco_do_gps(uint8_t tamanho) {
  Wire.beginTransmission(ENDERECO_GPS_I2C);
  Wire.write(REGISTRADOR_FLUXO_DADOS);
  if (Wire.endTransmission(false) != 0) return 0;

  const uint8_t recebidos = Wire.requestFrom(ENDERECO_GPS_I2C, tamanho);
  for (uint8_t i = 0; i < recebidos; i++) {
    gps.encode(Wire.read());
  }
  return recebidos;
}

void ler_buffer_do_gps() {
  uint16_t restante = ler_bytes_disponiveis();
  if (restante > LIMITE_BYTES_POR_LEITURA) restante = LIMITE_BYTES_POR_LEITURA;

  while (restante > 0) {
    const uint8_t tamanho = (restante > TAMANHO_BLOCO_I2C_BYTES)
                              ? TAMANHO_BLOCO_I2C_BYTES
                              : (uint8_t)restante;
    const uint8_t recebidos = ler_bloco_do_gps(tamanho);
    if (recebidos == 0) break;
    restante -= recebidos;
  }
}

// ---------------------------------------------------------------------------
// I2C: comandos UBX (so para modulos u-blox)
// ---------------------------------------------------------------------------
bool enviar_comando_ubx(uint8_t classe, uint8_t id,
                        const uint8_t* carga, uint8_t tamanho_carga) {
  if (tamanho_carga > TAMANHO_MAXIMO_CARGA_UBX_BYTES) return false;

  uint8_t pacote[TAMANHO_MAXIMO_CARGA_UBX_BYTES + 8];
  pacote[0] = 0xB5;
  pacote[1] = 0x62;
  pacote[2] = classe;
  pacote[3] = id;
  pacote[4] = tamanho_carga;
  pacote[5] = 0;
  memcpy(&pacote[6], carga, tamanho_carga);

  uint8_t soma_a = 0;
  uint8_t soma_b = 0;
  for (uint8_t i = 2; i < 6 + tamanho_carga; i++) {
    soma_a += pacote[i];
    soma_b += soma_a;
  }
  pacote[6 + tamanho_carga] = soma_a;
  pacote[7 + tamanho_carga] = soma_b;

  const uint8_t total = tamanho_carga + 8;
  uint8_t enviados = 0;
  while (enviados < total) {
    const uint8_t faltam = total - enviados;
    const uint8_t tamanho = (faltam > TAMANHO_BLOCO_I2C_BYTES)
                              ? TAMANHO_BLOCO_I2C_BYTES
                              : faltam;
    Wire.beginTransmission(ENDERECO_GPS_I2C);
    Wire.write(&pacote[enviados], tamanho);
    if (Wire.endTransmission() != 0) return false;
    enviados += tamanho;
  }
  return true;
}

bool configurar_taxa_de_atualizacao(uint8_t taxa_hz) {
  const uint16_t periodo_ms = 1000 / taxa_hz;
  const uint8_t carga[6] = {
    (uint8_t)(periodo_ms & 0xFF), (uint8_t)(periodo_ms >> 8),  // measRate
    1, 0,                                                      // navRate
    1, 0                                                       // timeRef = GPS
  };
  return enviar_comando_ubx(0x06, 0x08, carga, sizeof(carga)); // CFG-RATE
}

bool configurar_modelo_dinamico(uint8_t modelo) {
  uint8_t carga[36] = {0};
  carga[0] = 0x01;     // mascara: aplicar somente o modelo dinamico
  carga[2] = modelo;
  return enviar_comando_ubx(0x06, 0x24, carga, sizeof(carga)); // CFG-NAV5
}

// ---------------------------------------------------------------------------
// Logica de altitude
// ---------------------------------------------------------------------------
bool existe_altitude_nova() {
  return gps.altitude.isValid() && gps.altitude.isUpdated();
}

// Na rampa a referencia acompanha o GPS (media movel); depois do lancamento fica fixa
void atualizar_referencia(double altitude_gps_m, bool em_rampa) {
  if (!referencia_definida) {
    altitude_inicial_m  = altitude_gps_m;
    referencia_definida = true;
    return;
  }
  if (em_rampa) {
    altitude_inicial_m += FATOR_FILTRO_REFERENCIA * (altitude_gps_m - altitude_inicial_m);
  }
}

double calcular_altitude_relativa(double altitude_gps_m) {
  return fmax(0.0, altitude_gps_m - altitude_inicial_m);
}

void atualizar_velocidade_vertical(unsigned long agora_ms) {
  if (primeira_leitura) {
    primeira_leitura      = false;
    velocidade_vertical_ms = 0.0;
    variacao_vertical     = VariacaoVertical::ESTAVEL;
    if (DEBUG_GPS) Serial.println("GPS: primeira leitura recebida");
  } else {
    const double dt_s = (agora_ms - instante_anterior_ms) / 1000.0;
    if (dt_s <= 0.0) return;
    velocidade_vertical_ms = (altitude_absoluta_m - altitude_anterior_m) / dt_s;

    if (velocidade_vertical_ms >= LIMIAR_VELOCIDADE_VERTICAL_MS) {
      variacao_vertical = VariacaoVertical::SUBINDO;
    } else if (velocidade_vertical_ms <= -LIMIAR_VELOCIDADE_VERTICAL_MS) {
      variacao_vertical = VariacaoVertical::DESCENDO;
    } else {
      variacao_vertical = VariacaoVertical::ESTAVEL;
    }
  }
  altitude_anterior_m  = altitude_absoluta_m;
  instante_anterior_ms = agora_ms;
}

void atualizar_altitude_maxima() {
  if (altitude_relativa_m > altitude_maxima_m) {
    altitude_maxima_m = altitude_relativa_m;
  }
}

// Apogeu: altitude caiu QUEDA_APOGEU_M abaixo da maxima por N leituras seguidas
void atualizar_apogeu(bool em_rampa) {
  if (apogeu_detectado || em_rampa) return;

  if (altitude_maxima_m < ALTITUDE_MINIMA_APOGEU_M) {
    contagem_queda = 0;
    return;
  }

  if (altitude_maxima_m - altitude_relativa_m >= QUEDA_APOGEU_M) {
    contagem_queda++;
  } else {
    contagem_queda = 0;
  }

  if (contagem_queda >= LEITURAS_CONFIRMA_APOGEU) {
    apogeu_detectado = true;
    if (DEBUG_GPS) {
      Serial.print("GPS: apogeu detectado em ");
      Serial.print(altitude_maxima_m, 1);
      Serial.println(" m");
    }
  }
}

void anunciar_marcos() {
  while (altitude_relativa_m >= proximo_marco_m) {
    if (DEBUG_GPS) {
      Serial.print("Marco atingido: ");
      Serial.print(proximo_marco_m, 0);
      Serial.println(" metros acima do lancamento!");
    }
    proximo_marco_m += PASSO_MARCO_M;
  }
}

// ---------------------------------------------------------------------------
// Preenchimento de dados (posicao, data e hora)
// ---------------------------------------------------------------------------
void preencher_posicao(DadosGps& dados) {
  dados.fix_valido      = gps.location.isValid();
  dados.latitude_graus  = dados.fix_valido ? gps.location.lat() : 0.0;
  dados.longitude_graus = dados.fix_valido ? gps.location.lng() : 0.0;
  dados.satelites       = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
}

void preencher_data_hora(DadosGps& dados) {
  dados.data_hora_valida = gps.date.isValid() && gps.time.isValid();
  dados.ano     = dados.data_hora_valida ? gps.date.year()   : 0;
  dados.mes     = dados.data_hora_valida ? gps.date.month()  : 0;
  dados.dia     = dados.data_hora_valida ? gps.date.day()    : 0;
  dados.hora    = dados.data_hora_valida ? gps.time.hour()   : 0;
  dados.minuto  = dados.data_hora_valida ? gps.time.minute() : 0;
  dados.segundo = dados.data_hora_valida ? gps.time.second() : 0;
}

void preencher_altitudes(DadosGps& dados) {
  dados.altitude_absoluta_m    = altitude_absoluta_m;
  dados.altitude_inicial_m     = altitude_inicial_m;
  dados.altitude_relativa_m    = altitude_relativa_m;
  dados.altitude_maxima_m      = altitude_maxima_m;
  dados.velocidade_vertical_ms = velocidade_vertical_ms;
  dados.variacao               = variacao_vertical;
  dados.apogeu_detectado       = apogeu_detectado;
}

const char* texto_variacao(VariacaoVertical variacao) {
  switch (variacao) {
    case VariacaoVertical::SUBINDO:  return "SUBINDO";
    case VariacaoVertical::DESCENDO: return "DESCENDO";
    default:                         return "ESTAVEL";
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// API publica
// ---------------------------------------------------------------------------
bool gps_esta_conectado() {
  Wire.beginTransmission(ENDERECO_GPS_I2C);
  return Wire.endTransmission() == 0;
}

void gps_resetar_voo() {
  referencia_definida    = false;
  altitude_inicial_m     = 0.0;
  altitude_absoluta_m    = 0.0;
  altitude_anterior_m    = 0.0;
  altitude_relativa_m    = 0.0;
  altitude_maxima_m      = 0.0;
  velocidade_vertical_ms = 0.0;
  proximo_marco_m        = PASSO_MARCO_M;
  variacao_vertical      = VariacaoVertical::ESTAVEL;
  primeira_leitura       = true;
  apogeu_detectado       = false;
  contagem_queda         = 0;
  instante_anterior_ms   = 0;
  ultima_leitura_ms      = 0;
}

bool gps_iniciar() {
  Wire.begin(PINO_SDA_I2C, PINO_SCL_I2C);
  Wire.setClock(CLOCK_I2C_HZ);
  gps_resetar_voo();

  if (!gps_esta_conectado()) {
    Serial.println("GPS: modulo nao responde no I2C (endereco 0x42)");
    return false;
  }

  if (CONFIGURAR_MODULO_NO_INICIO) {
    const bool taxa_ok   = configurar_taxa_de_atualizacao(TAXA_ATUALIZACAO_HZ);
    const bool modelo_ok = configurar_modelo_dinamico(MODELO_DINAMICO_AEREO_4G);
    if (DEBUG_GPS) {
      Serial.print("GPS: taxa ");
      Serial.println(taxa_ok ? "configurada" : "FALHOU");
      Serial.print("GPS: modelo aereo ");
      Serial.println(modelo_ok ? "configurado" : "FALHOU");
    }
  }

  Serial.println("GPS iniciado. Aguardando sinal...");
  return true;
}

bool gps_atualizar(bool em_rampa) {
  ler_buffer_do_gps();

  if (!existe_altitude_nova()) return false;

  // Sem referencia da rampa, a altitude relativa nao tem sentido
  if (!referencia_definida && !em_rampa) {
    gps.altitude.meters();  // consome a atualizacao
    return false;
  }

  const unsigned long agora_ms = millis();
  altitude_absoluta_m = gps.altitude.meters();

  atualizar_referencia(altitude_absoluta_m, em_rampa);
  altitude_relativa_m = calcular_altitude_relativa(altitude_absoluta_m);

  atualizar_velocidade_vertical(agora_ms);
  atualizar_altitude_maxima();
  atualizar_apogeu(em_rampa);
  anunciar_marcos();

  ultima_leitura_ms = agora_ms;
  return true;
}

DadosGps gps_obter_dados() {
  DadosGps dados;
  preencher_posicao(dados);
  preencher_data_hora(dados);
  preencher_altitudes(dados);
  return dados;
}

double           gps_altitude_relativa_m()     { return altitude_relativa_m; }
double           gps_altitude_maxima_m()       { return altitude_maxima_m; }
double           gps_velocidade_vertical_ms()  { return velocidade_vertical_ms; }
VariacaoVertical gps_variacao_vertical()       { return variacao_vertical; }
bool             gps_apogeu_detectado()        { return apogeu_detectado; }
bool             gps_tem_fix()                 { return gps.location.isValid(); }
bool             gps_referencia_definida()     { return referencia_definida; }

bool gps_dados_recentes() {
  return ultima_leitura_ms != 0 && (millis() - ultima_leitura_ms) < TIMEOUT_DADOS_MS;
}

void gps_definir_altitude_inicial(double altitude_m) {
  altitude_inicial_m  = altitude_m;
  referencia_definida = true;
}

void gps_imprimir_status() {
  Serial.print("Altitude atual: ");
  Serial.print(altitude_relativa_m, 2);
  Serial.print(" m | Maxima: ");
  Serial.print(altitude_maxima_m, 2);
  Serial.print(" m | Vel. vertical: ");
  Serial.print(velocidade_vertical_ms, 1);
  Serial.print(" m/s | ");
  Serial.println(texto_variacao(variacao_vertical));
}
