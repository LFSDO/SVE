#pragma once
#include <Arduino.h>

/*
  Modulo GPS (I2C) para o computador de bordo.
  Convenção de nomes: snake_case em português sem acento; constantes em
  MAIUSCULO_SNAKE com sufixo da unidade; tipos PascalCase.

  Uso no projeto central:
    setup():  gps_iniciar();
    loop():   gps_atualizar(estado_atual == EstadoVoo::RAMPA);
              if (gps_apogeu_detectado()) { ... }
*/

enum class VariacaoVertical {
  SUBINDO,
  DESCENDO,
  ESTAVEL
};

// Pacote com tudo que o projeto central pode precisar (ex.: gravar no SD)
struct DadosGps {
  bool     fix_valido;
  uint8_t  satelites;
  double   latitude_graus;
  double   longitude_graus;
  double   altitude_absoluta_m;     // acima do nivel do mar (GPS)
  double   altitude_inicial_m;      // referencia da rampa
  double   altitude_relativa_m;     // acima do local de lancamento
  double   altitude_maxima_m;
  double   velocidade_vertical_ms;
  VariacaoVertical variacao;
  bool     apogeu_detectado;
  bool     data_hora_valida;        // data e hora em UTC
  uint16_t ano;
  uint8_t  mes;
  uint8_t  dia;
  uint8_t  hora;
  uint8_t  minuto;
  uint8_t  segundo;
};

// --- Inicializacao ---------------------------------------------------------
bool gps_iniciar();                       // I2C + configuracao do modulo (nao trava)
bool gps_esta_conectado();                // o modulo responde no barramento?
void gps_resetar_voo();                   // zera maxima, apogeu, marcos e referencia

// --- Ciclo principal (chamar em todo loop) ---------------------------------
// em_rampa = true enquanto o foguete esta parado na rampa: a referencia de
// altitude e calibrada nesse periodo e o apogeu fica desarmado.
// Retorna true quando chegou uma altitude nova.
bool gps_atualizar(bool em_rampa);

// --- Consultas -------------------------------------------------------------
DadosGps         gps_obter_dados();
double           gps_altitude_relativa_m();
double           gps_altitude_maxima_m();
double           gps_velocidade_vertical_ms();
VariacaoVertical gps_variacao_vertical();
bool             gps_apogeu_detectado();
bool             gps_tem_fix();
bool             gps_referencia_definida();
bool             gps_dados_recentes();    // chegou dado novo nos ultimos segundos?

// --- Ajustes manuais -------------------------------------------------------
void gps_definir_altitude_inicial(double altitude_m);  // usar se nao houve fix na rampa

// --- Depuracao -------------------------------------------------------------
void gps_imprimir_status();
