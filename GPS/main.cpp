#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "gps_voo.h"

/*
  Convenção de nomes:
   - funções e variáveis: snake_case em português, sem acento
   - constantes: MAIUSCULO_SNAKE em português, com sufixo da unidade (_MS, _MS2, _C)
   - tipos: PascalCase; valores de enum: MAIUSCULO

  TODO:
   - [] Refazer o sistema de display de altitude
   - [] Guardar os dados em SD (usar gps_obter_dados())
   - [] Acionar o paraquedas (apogeu: gps_apogeu_detectado(), idealmente junto do barometro)
   - [] Estados DESCIDA -> POUSO -> REPOUSO
   - [x] Implementar os estados de voo
   - [x] Sem travas: nada usa while bloqueante nem delay
*/

// ---------------------------------------------------------------------------
// Configuração
// ---------------------------------------------------------------------------
constexpr unsigned long TAXA_BAUD             = 115200;
constexpr float         LIMIAR_DECOLAGEM_MS2  = 19.6f;   // 2 g (faixa do sensor: +-16 g)
constexpr float         TEMPERATURA_CRITICA_C = 35.0f;
constexpr unsigned long INTERVALO_LOOP_MS     = 10;
constexpr unsigned long INTERVALO_STATUS_MS   = 1000;
constexpr uint8_t       TENTATIVAS_INICIAR_MPU = 5;

// ---------------------------------------------------------------------------
// Tipos
// ---------------------------------------------------------------------------
enum class EstadoVoo {
  RAMPA,
  QUEIMA,
  COASTING,
  DESCIDA,
  POUSO,
  REPOUSO
};

struct LeituraMpu {
  float aceleracao_z_ms2;
  float temperatura_c;
};

// ---------------------------------------------------------------------------
// Variáveis globais
// ---------------------------------------------------------------------------
Adafruit_MPU6050 mpu;

EstadoVoo     estado_atual  = EstadoVoo::RAMPA;
bool          mpu_ok        = false;
unsigned long inicio_subida = 0;

// ---------------------------------------------------------------------------
// Utilidades
// ---------------------------------------------------------------------------
// Retorna true quando passou o intervalo (substitui delay, sem travar)
bool passou_intervalo(unsigned long& ultimo_ms, unsigned long intervalo_ms) {
  const unsigned long agora_ms = millis();
  if (agora_ms - ultimo_ms < intervalo_ms) return false;
  ultimo_ms = agora_ms;
  return true;
}

const char* nome_estado(EstadoVoo estado) {
  switch (estado) {
    case EstadoVoo::RAMPA:    return "RAMPA";
    case EstadoVoo::QUEIMA:   return "QUEIMA";
    case EstadoVoo::COASTING: return "COASTING";
    case EstadoVoo::DESCIDA:  return "DESCIDA";
    case EstadoVoo::POUSO:    return "POUSO";
    default:                  return "REPOUSO";
  }
}

void mudar_estado(EstadoVoo novo_estado) {
  if (novo_estado == estado_atual) return;
  estado_atual = novo_estado;
  Serial.print("Estado de voo: ");
  Serial.println(nome_estado(estado_atual));
}

// ---------------------------------------------------------------------------
// Sensor MPU6050
// ---------------------------------------------------------------------------
bool inicializar_sensor() {
  for (uint8_t tentativa = 0; tentativa < TENTATIVAS_INICIAR_MPU; tentativa++) {
    if (mpu.begin()) {
      mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
      Serial.println("MPU6050 Initialized");
      return true;
    }
    Serial.println("ERROR INITIALIZING MPU6050");
    delay(100);  // so no setup, antes do voo
  }
  return false;
}

// Uma unica leitura por ciclo, compartilhada por todas as funcoes
LeituraMpu ler_mpu() {
  sensors_event_t evento_acel, evento_giro, evento_temp;
  mpu.getEvent(&evento_acel, &evento_giro, &evento_temp);

  LeituraMpu leitura;
  leitura.aceleracao_z_ms2 = evento_acel.acceleration.z;
  leitura.temperatura_c    = evento_temp.temperature;
  return leitura;
}

// ---------------------------------------------------------------------------
// Verificações de cada fase
// ---------------------------------------------------------------------------
void verificar_decolagem(const LeituraMpu& leitura) {
  if (leitura.aceleracao_z_ms2 < LIMIAR_DECOLAGEM_MS2) return;

  Serial.println("Aceleração vertical (z) acima do limiar de decolagem: Inicio de voo detectado.");
  inicio_subida = millis();
  mudar_estado(EstadoVoo::QUEIMA);
}

void registrar_fim_da_subida(float aceleracao_z) {
  const unsigned long tempo_subida = millis() - inicio_subida;

  Serial.print("Detectada aceleracao vertical negativa: ");
  Serial.println(aceleracao_z, 1);
  Serial.print("Tempo de subida: ");
  Serial.print(tempo_subida / 1000);
  Serial.println(" segundos");
}

// Fim da queima (nao detecta o apogeu, quem faz isso e o GPS/barometro)
void verificar_fim_da_queima(const LeituraMpu& leitura) {
  if (leitura.aceleracao_z_ms2 >= 0) return;

  registrar_fim_da_subida(leitura.aceleracao_z_ms2);
  mudar_estado(EstadoVoo::COASTING);
}

void verificar_temperatura(const LeituraMpu& leitura) {
  if (leitura.temperatura_c <= TEMPERATURA_CRITICA_C) return;

  Serial.print("TEMPERATURA CRITICA EXCEDIDA!: ");
  Serial.println(leitura.temperatura_c, 1);
}

// O GPS avisa o apogeu e o estado de voo avanca
void verificar_apogeu() {
  if (!gps_apogeu_detectado()) return;

  Serial.println("Apogeu detectado pelo GPS.");
  // TODO: acionar o paraquedas aqui
  mudar_estado(EstadoVoo::DESCIDA);
}

void executar_estado(const LeituraMpu& leitura) {
  switch (estado_atual) {
    case EstadoVoo::RAMPA:
      verificar_decolagem(leitura);
      break;
    case EstadoVoo::QUEIMA:
      verificar_fim_da_queima(leitura);
      verificar_temperatura(leitura);
      break;
    case EstadoVoo::COASTING:
      verificar_apogeu();
      break;
    default:
      break;   // DESCIDA, POUSO e REPOUSO ainda nao implementados
  }
}

// ---------------------------------------------------------------------------
// Status periódico
// ---------------------------------------------------------------------------
void imprimir_status() {
  Serial.print("[");
  Serial.print(nome_estado(estado_atual));
  Serial.print("] ");
  gps_imprimir_status();
}

// ---------------------------------------------------------------------------
// RUN (deve ser integrado ao main.cpp central: mover o conteudo para o setup/loop de la)
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(TAXA_BAUD);
  Serial.println("ESP32 Inicializado");

  mpu_ok = inicializar_sensor();
  if (!mpu_ok) Serial.println("MPU6050 indisponivel: voo sem deteccao de aceleracao!");

  if (!gps_iniciar()) Serial.println("GPS indisponivel: voo sem altitude GPS!");
}

void loop() {
  static unsigned long ultimo_ciclo_ms  = 0;
  static unsigned long ultimo_status_ms = 0;

  // O GPS e lido em todo ciclo (inclusive na rampa, para calibrar a altitude inicial)
  gps_atualizar(estado_atual == EstadoVoo::RAMPA);

  if (mpu_ok && passou_intervalo(ultimo_ciclo_ms, INTERVALO_LOOP_MS)) {
    executar_estado(ler_mpu());
  }

  if (passou_intervalo(ultimo_status_ms, INTERVALO_STATUS_MS)) {
    imprimir_status();
  }
}
