/* Bibliotecas Wifi */
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

/* Biblioteca para gravar dados no microSD */
#include <SPI.h> //INCLUSÃO DE BIBLIOTECA
#include <SD.h>
/* Biblioteca complementar (String) */
#include<stdlib.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

/* Biblioteca do RTC */
#include <RtcDS3231.h>
RtcDS3231<TwoWire> Rtc(Wire);

/* Biblioteca do sensor de Temperatura e Umidade*/
#include "ClosedCube_HDC1080.h"

/* Pino de leitra das RPM do Anemometro */
#define WindPin D8

/* Constante do microSD*/
const int chipSelect = SS; //PINO DIGITAL UTILIZADO PELO TERMINAL CS DO MÓDULO

// -- Variaveis do RTC
RtcDateTime dt;

// -- Variavel para conta as gravações do cartao microSD
int cont;

// -- Variavel para construção da string a ser salva no cartao microSD
char sLinhaDados[100];

// -- Variaveis Climatologicas maximas e minimas
float tempMax;
float tempMin;
float umdMax;
float umdMin;
float presMax;
float presMin;

/* Constantes do BPW34 (Rn)*/
#define BPW34pin        A0
#define samples         16

unsigned int sample[samples];

unsigned char s = 0;
unsigned int avg = 0;
unsigned long sum = 0;
float irValue = 0;

// -- Constantes
const float pi = 3.1415;          //Número de pi
int period = 5000;                //Tempo de medida(miliseconds)
int delaytime = 10000;            //Invervalo entre as amostras (miliseconds)
int radius = 105;                 //Raio do anemometro(mm)

// -- Variáveis Globais --
unsigned int Sample  = 0;        //Armazena o número de amostras
volatile int counter = 0;        //Contador para o sensor
unsigned int RPM = 0;            //Rotações por minuto
float windspeed = 0;             //Velocidade do vento (m/s)

// -- Irrigação (Kc)
float KcAlface[] = {0.5, 0.7, 0.95, 0.9};  //inicio, desenvolvimento, reprodutivo, maturacao - 35 dias
float KcRucula[] = {0.4, 0.7, 0.95, 0.75}; //inicio, desenvolvimento, reprodutivo, maturacao - 35 dias
float KcEspinafre[] = {0.4, 0.7, 0.95, 0.9}; //40 dias
float KcBeterraba[] = {0.4, 0.75, 1.05, 0.6}; //50 dias

// -- Profundidade de enraizamento (Zr)
float ZrAlface[] = {0.1, 0.2, 0.25, 0.35};  //inicio, desenvolvimento, reprodutivo, maturacao - 35 dias
float ZrRucula[] = {0.1, 0.2, 0.25, 0.35}; //inicio, desenvolvimento, reprodutivo, maturacao - 35 dias
float ZrEspinafre[] = {0.1, 0.2, 0.3, 0.4}; //40 dias
float ZrBeterraba[] = {0.2, 0.35, 0.5, 0.7}; //50 dias

// -- Fração de TAW que pode ser esgotada entes de entrar no estresse de umidade (p)
float pAlface = 0.3;
float pRucula = 0.3;
float pEspinafre = 0.2;
float pBeterraba = 0.5;

// -- Capacidade de campo e Ponto de murcha
float Qfc = 0.4; // cc
float Qwp = 0.1; // pm

// -- Etc acumulado
float EtcacumAlface = 0;
float EtcacumRucula = 0;
float EtcacumEspinafre = 0;
float EtcacumBeterraba = 0;

// -- medias diarias
float temperatura;
float umidade;
float radiacao;
float vento;
float pressao;

int dias = 0;

// -- reles
int pino_rele1 = D2;
int pino_rele2 = D3;
int pino_rele3 = D4;
int pino_rele4 = D5;

// -- Especificações do Sistema de Irrigação
int Ei = 95; //Eficiência de aplicação de água do sistema de irrigação, %
int Np = 24480; // Número de plantas por hectares.
float Ne = 1; // número de emissores por planta.
float qe = 1.5;// vazão do emissor, L/h.

// -- Variavel do sensor de Temperatura e Umidade
ClosedCube_HDC1080 hdc1080;

//Configurar conexão do sensor
Adafruit_BMP280 bmp; // I2C

// -- Token de autenticacao do Blynk App.
char auth[] = "c79cabcb5c3a460694efd96266d9e0c7";

// -- Login e Senha da rede wifi
char ssid[] = "Jampierre’s iPhone";
char pass[] = "jampierre32";

// -- Taxa de transmissao:
#define ESP8266_BAUD 115200

BlynkTimer timer;

void setup() {
  Serial.begin(9600);

  pinMode(chipSelect, OUTPUT); //DEFINE O PINO COMO SAÍDA

  if (!SD.begin(chipSelect)) { //SE O CARTÃO DE MEMÓRIA NÃO ESTIVER PRESENTE OU FALHAR, FAZ
    return; //NÃO FAZ MAIS NADA
  }

  pinMode(pino_rele1, OUTPUT);
  pinMode(pino_rele2, OUTPUT);
  pinMode(pino_rele3, OUTPUT);
  pinMode(pino_rele4, OUTPUT);

  //desliga os reles
  digitalWrite(pino_rele1, HIGH);
  digitalWrite(pino_rele2, HIGH);
  digitalWrite(pino_rele3, HIGH);
  digitalWrite(pino_rele4, HIGH);

  memset(sample, 0, samples); //Limpa a variavel

  bmp.begin();    //Inicializa o sensor

  Rtc.Begin();

  //hora atual de quando o sistema iniciou
  dt = Rtc.GetDateTime();
  cont = dt.Hour();

  // Configuracao padrao:
  //  - Heater off
  //  - 14 bit Resolucao da medicao de temperatura e umidade
  hdc1080.begin(0x40);

  pinMode(WindPin, INPUT_PULLUP);  //configura o digital 8 como entrada
  digitalWrite(WindPin, HIGH);    //pull-up interno ativo

  inicializacaoVarClimaticasMaxMin();

  inicializaVariaveisPMFAO();

  // Configura a funcao para ser chamada a cada 8s
  timer.setInterval(8000L, sendSensor);
}

void inicializacaoVarClimaticasMaxMin() {
  tempMax = hdc1080.readTemperature();
  tempMin = tempMax;
  umdMax = hdc1080.readHumidity();
  umdMin = umdMax;
  presMax = bmp.readPressure();
  presMin = presMax;
}

void inicializaVariaveisPMFAO() {
  delay(3000);
  temperatura = hdc1080.readTemperature();
  umidade = hdc1080.readHumidity();
  radiacao = ((irValue * 86400) / 1000) / 1000;

  windvelocity();
  RPMcalc();
  WindSpeed();
  vento = windspeed;

  pressao = bmp.readPressure();
}

void funcaoConstroiStringDados(char* sDados, char* sHoraDia) {
  char   sAux[10];

  strcat(sDados, sHoraDia);   //String sHoraDia eh concatenada em sDados
  strcat(sDados, "\t");       //String "\t" eh concatenada em sDados

  /* Temperatura Inst. Max. e Min */
  dtostrf(hdc1080.readTemperature(), 1, 1, sAux); //DoubleToStringFunction: Valor Temp Inst. eh copiado para a String
  strcat(sDados, sAux);       //String SAux eh concatenada para sDados
  strcat(sDados, "\t");       //String "\t" eh concatenada em sDados

  dtostrf(tempMax, 1, 1, sAux); //DoubleToStringFunction: Valor Temp Max. eh copiado para a String
  strcat(sDados, sAux);       //String SAux eh concatenada para sDados
  strcat(sDados, "\t");       //String "\t" eh concatenada em sDados

  dtostrf(tempMin, 1, 1, sAux); //DoubleToStringFunction: Valor Temp Min. eh copiado para a String
  strcat(sDados, sAux);       //String SAux eh concatenada para sDados
  strcat(sDados, "\t");       //String "\t" eh concatenada em sDados

  /* Umidade Inst. Max. e Min */
  dtostrf(hdc1080.readHumidity(), 1, 1, sAux); //Valor de Umd Inst. eh copiado para a String
  strcat(sDados, sAux);        //String SAux eh concatenada em sDados
  strcat(sDados, "\t");        //String "\t" é concatenada em sDados

  dtostrf(umdMax, 1, 1, sAux); //Valor de Umd Max. eh copiado para a String
  strcat(sDados, sAux);        //String SAux eh concatenada em sDados
  strcat(sDados, "\t");        //String "\t" é concatenada em sDados

  dtostrf(umdMin, 1, 1, sAux); //Valor de Umd Min. eh copiado para a String
  strcat(sDados, sAux);        //String SAux eh concatenada em sDados
  strcat(sDados, "\t");        //String "\t" é concatenada em sDados

  /* Pressão Inst. Max. Min*/
  dtostrf(bmp.readPressure() / 100.0F, 1, 1, sAux); //Valor de Press Inst. eh copiado para a String
  strcat(sDados, sAux);        //String SAux eh concatenada em sDados
  strcat(sDados, "\t");        //String "\t" é concatenada em sDados

  dtostrf(presMax / 100.0F, 1, 1, sAux); //Valor de Press Max. eh copiado para a String
  strcat(sDados, sAux);        //String SAux eh concatenada em sDados
  strcat(sDados, "\t");        //String "\t" é concatenada em sDados

  dtostrf(presMin / 100.0F, 1, 1, sAux); //Valor de Press Min. eh copiado para a String
  strcat(sDados, sAux);        //String SAux eh concatenada em sDados
  strcat(sDados, "\t");        //String "\t" é concatenada em sDados

  /* Velocidade do Vento*/
  dtostrf(windspeed, 1, 1, sAux); //Valor de Velc Vento eh copiado para a String
  strcat(sDados, sAux);        //String SAux eh concatenada em sDados
  strcat(sDados, "\t\t");        //String "\t" é concatenada em sDados

  /* Radiação Solar*/
  dtostrf((irValue * 86400) / 1000, 1, 2, sAux); //Valor de Rn eh copiado para a String
  strcat(sDados, sAux);        //String SAux eh concatenada em sDados
  strcat(sDados, "\t");        //String "\t" é concatenada em sDados

  /* Temperatura Interna do Sistema*/
  dtostrf(Rtc.GetTemperature().AsFloatDegC(), 1, 1, sAux); //Valor de Temp. Interna eh copiado para a String
  strcat(sDados, sAux);        //String SAux eh concatenada em sDados
}

void sendSensor()
{
  Blynk.virtualWrite(V0, hdc1080.readTemperature());
  Blynk.virtualWrite(V1, hdc1080.readHumidity());
  Blynk.virtualWrite(V11, bmp.readPressure());
  Blynk.virtualWrite(V13, Rtc.GetTemperature().AsFloatDegC());

  Blynk.virtualWrite(V10, (irValue * 86400) / 1000); // kJ/m2dia1
  Blynk.virtualWrite(V8, windspeed); // Vel. Vento:  [m/s]
}

void loop() {
  Blynk.run();
  timer.run();

  /* Inicio Rn*/
  sum -= sample[s];
  sample[s] = analogRead(BPW34pin);
  sum += sample[s];
  s += 1;

  if (s >= samples)
  {
    s = 0;
    avg = (sum >> 4);
    irValue = ((16.3366 * avg) - 30.69);
    irValue *= 0.0079;
    if (irValue < 0)
    {
      irValue = 0;
    }
  }
  /* Fim Rn*/

  /* Velocidade do vento*/
  windvelocity();
  RPMcalc();
  WindSpeed();

  /* Coleta das maximas e minimas climatologicas*/
  if (tempMin > hdc1080.readTemperature()) {
    tempMin = hdc1080.readTemperature();
  }
  if (tempMax < hdc1080.readTemperature()) {
    tempMax = hdc1080.readTemperature();
  }
  if (umdMin > hdc1080.readHumidity()) {
    umdMin = hdc1080.readHumidity();
  }
  if (umdMax < hdc1080.readHumidity()) {
    umdMax = hdc1080.readHumidity();
  }
  if (presMin > bmp.readPressure()) {
    presMin = bmp.readPressure();
  }
  if (presMax < bmp.readPressure()) {
    presMax = bmp.readPressure();
  }

  /* Gera registro no cartao de memoria*/
  dt = Rtc.GetDateTime();
  if (cont == dt.Hour()) {
    datalogger();
    temperatura = (temperatura + hdc1080.readTemperature()) / 2;
    umidade = (umidade + hdc1080.readHumidity()) / 2;
    radiacao = (radiacao + (((irValue * 86400) / 1000) / 1000)) / 2;
    vento = (vento + windspeed) / 2;
    pressao = (pressao + bmp.readPressure()) / 2;

    if (cont > 23) {
      cont = 0;
      double tempo;
      float ET0, ETc, TAW, RAW;
      int i;
      //germinaçao do alface 4 a 7 dias
      if (dias <= 4) {
        i = 0;
      }
      if (dias > 4 && dias <= 14) {
        i = 1;
      }
      if (dias > 14 && dias <= 35) {
        i = 2;
      }
      if (dias > 35) {
        i = 3;
      }

      ET0 = eqET0(temperatura, pressao, vento, umidade, radiacao);
      ETc = eqETc(KcAlface[i], ET0);

      EtcacumAlface += ETc;

      TAW = eqTAW(ZrAlface[i], Qfc, Qwp);
      RAW = eqRAW(TAW, pAlface);

      logSituacaoCultura(TAW, RAW, "Alface", ET0, ETc, EtcacumAlface, dias); // escrever funcão

      if (EtcacumAlface > RAW * 0.8) {
        //        if (EtcacumAlface > RAW) {
        //          //irriga RAW
        //          tempo = tempoDeIrrigacao(Ei, RAW, Np, Ne, qe);
        //          EtcacumAlface -= RAW;
        //          logIrrigacao(tempo, "Alface", RAW, EtcacumAlface);
        //        } else {
        //          // irriga EtcacumAlface
        //          tempo = tempoDeIrrigacao(Ei, EtcacumAlface, Np, Ne, qe);
        //          EtcacumAlface = 0;
        //          logIrrigacao(tempo, "Alface-irrigacao", EtcacumAlface, EtcacumAlface);
        //        }
        tempo = tempoDeIrrigacao(Ei, EtcacumAlface, Np, Ne, qe);
        EtcacumAlface = 0;
        logIrrigacao(tempo, "Alface-irrigacao", EtcacumAlface, EtcacumAlface);
        digitalWrite(pino_rele1, LOW);
        delay(tempo);
        digitalWrite(pino_rele1, HIGH);
      }


      //germinaçao da...
      if (dias <= 4) {
        i = 0;
      }
      if (dias > 4 && dias <= 14) {
        i = 1;
      }
      if (dias > 14 && dias <= 35) {
        i = 2;
      }
      if (dias > 35) {
        i = 3;
      }

      ETc = eqETc(KcRucula[i], ET0);

      EtcacumRucula += ETc;

      TAW = eqTAW(ZrRucula[i], Qfc, Qwp);
      RAW = eqRAW(TAW, pRucula);

      logSituacaoCultura(TAW, RAW, "Rucula", ET0, ETc, EtcacumRucula, dias); // escrever funcão

      if (EtcacumRucula > RAW * 0.8) {
        //        if (EtcacumRucula > RAW) {
        //          //irriga RAW
        //          tempo = tempoDeIrrigacao(Ei, RAW, Np, Ne, qe);
        //          EtcacumRucula -= RAW;
        //          logIrrigacao(tempo, "Rucula", RAW, EtcacumRucula);
        //        } else {
        //          // irriga EtcacumRucula
        //          tempo = tempoDeIrrigacao(Ei, EtcacumRucula, Np, Ne, qe);
        //          EtcacumRucula = 0;
        //          logIrrigacao(tempo, "Rucula-irrigacao", EtcacumRucula, EtcacumRucula);
        //        }
        tempo = tempoDeIrrigacao(Ei, EtcacumRucula, Np, Ne, qe);
        EtcacumRucula = 0;
        logIrrigacao(tempo, "Rucula-irrigacao", EtcacumRucula, EtcacumRucula);
        digitalWrite(pino_rele2, LOW);
        delay(tempo);
        digitalWrite(pino_rele2, HIGH);
      }

      //germinaçao da...
      if (dias <= 4) {
        i = 0;
      }
      if (dias > 4 && dias <= 14) {
        i = 1;
      }
      if (dias > 14 && dias <= 35) {
        i = 2;
      }
      if (dias > 35) {
        i = 3;
      }

      ETc = eqETc(KcEspinafre[i], ET0);

      EtcacumEspinafre += ETc;

      TAW = eqTAW(ZrEspinafre[i], Qfc, Qwp);
      RAW = eqRAW(TAW, pEspinafre);

      logSituacaoCultura(TAW, RAW, "Espinafre", ET0, ETc, EtcacumEspinafre, dias);

      if (EtcacumEspinafre > RAW * 0.8) {
        //        if (EtcacumEspinafre > RAW) {
        //          //irriga RAW
        //          tempo = tempoDeIrrigacao(Ei, RAW, Np, Ne, qe);
        //          EtcacumEspinafre -= RAW;
        //          logIrrigacao(tempo, "Espinafre", RAW, EtcacumEspinafre);
        //        } else {
        //          // irriga EtcacumEspinafre
        //          tempo = tempoDeIrrigacao(Ei, EtcacumEspinafre, Np, Ne, qe);
        //          EtcacumEspinafre = 0;
        //          logIrrigacao(tempo, "Espinafre-irrigacao", EtcacumEspinafre, EtcacumEspinafre);
        //        }
        tempo = tempoDeIrrigacao(Ei, EtcacumEspinafre, Np, Ne, qe);
        EtcacumEspinafre = 0;
        logIrrigacao(tempo, "Espinafre-irrigacao", EtcacumEspinafre, EtcacumEspinafre);
        digitalWrite(pino_rele3, LOW);
        delay(tempo);
        digitalWrite(pino_rele3, HIGH);
      }

      //germinaçao d...
      if (dias <= 4) {
        i = 0;
      }
      if (dias > 4 && dias <= 14) {
        i = 1;
      }
      if (dias > 14 && dias <= 35) {
        i = 2;
      }
      if (dias > 35) {
        i = 3;
      }

      ETc = eqETc(KcBeterraba[i], ET0);

      EtcacumEspinafre += ETc;

      TAW = eqTAW(ZrBeterraba[i], Qfc, Qwp);
      RAW = eqRAW(TAW, pBeterraba);

      logSituacaoCultura(TAW, RAW, "Beterraba", ET0, ETc, EtcacumBeterraba, dias);

      if (EtcacumBeterraba > RAW * 0.8) {
        //        if (EtcacumBeterraba > RAW) {
        //          //irriga RAW
        //          tempo = tempoDeIrrigacao(Ei, RAW, Np, Ne, qe);
        //          EtcacumBeterraba -= RAW;
        //          logIrrigacao(tempo, "Beterraba", RAW, EtcacumBeterraba);
        //        } else {
        //          // irriga EtcacumBeterraba
        //          tempo = tempoDeIrrigacao(Ei, EtcacumBeterraba, Np, Ne, qe);
        //          EtcacumBeterraba = 0;
        //          logIrrigacao(tempo, "Beterraba-irrigacao", EtcacumBeterraba, EtcacumBeterraba);
        //        }
        tempo = tempoDeIrrigacao(Ei, EtcacumBeterraba, Np, Ne, qe);
        EtcacumBeterraba = 0;
        logIrrigacao(tempo, "Beterraba-irrigacao", EtcacumBeterraba, EtcacumBeterraba);
        digitalWrite(pino_rele4, LOW);
        delay(tempo);
        digitalWrite(pino_rele4, HIGH);
      }
      dias++;
      inicializaVariaveisPMFAO();
    } else {
      cont++;
    }
  }

  delay(delaytime);//taxa de atualização
}

float eqTAW(float Zr, float Qfc, float Qwp) {
  return 1000 * (Qfc - Qwp) * Zr;
}

float eqRAW(float TAW, float p) {
  return p * TAW;
}

float eqET0(float t, float p, float v, float ur, float rn) {

  /* Todas as variaveis estao em media diaria*/
  // t = Temperatura (C)

  /*var temporarias*/
  float tmp1;
  float tmp2;
  float tmp3;
  float tmp4;
  float tmp5;

  // es = Pressão de saturação do vapor d'água (kPa)
  tmp1 = (17.5 * t) / (t + 237.3);
  float es = 0.6108 * exp(tmp1);

  //s = Declividade da curva de pressão de saturação do vapor d'água (kPa C-1)
  tmp1 = 4098 * es;
  float s = tmp1 / pow((t + 237.3), 2);

  // lambda = Calor latente de evaporação (Mj Kg-1)
  tmp1 = 2.361 * pow(10, -3);
  tmp2 = tmp1 - t;
  float lambda = 2.501 - tmp2;

  // p = media diaria (kPa)
  p /= 10;

  //psy = Coeficiente psicrométrico (kPa C-1)
  float psy = 0.0016286 * (p / lambda);

  //v = velocidade do tempo (ms-1)

  //ur = umidade relativa do ar (%)

  //ea = Pressão parcial de vapor d'água (kPa)
  float ea = es * ur / 100;

  //rn = Saldo de radiação (Mj m-2d-1)

  //g = fluxo do calor do solo (Mj m-2d-1)
  float g = 0;

  tmp1 = v * (es - ea);
  tmp2 = ((psy * 900) / (s + psy) + (t + 273)) * tmp1;
  tmp3 = s / (s + psy);
  tmp4 = rn - g;
  tmp5 = 1 / lambda;

  //et0 = Evapotranspiração de referencia (mmd-1)
  float et0 = tmp3 * tmp4 * tmp5 + tmp2;

  return et0;
}

float eqETc(float Kc, float Et0) {
  return Kc * Et0;
}

//Função para medir velocidade do vento
void windvelocity() {
  windspeed = 0;

  counter = 0;
  attachInterrupt(digitalPinToInterrupt(WindPin), addcount, RISING);
  unsigned long millis();
  long startTime = millis();
  while (millis() < startTime + period) {
    yield();
  }
}

//Função para calcular o RPM
void RPMcalc() {
  RPM = ((counter) * 60) / (period / 1000); // Calculate revolutions per minute (RPM)
}

//Velocidade do vento em m/s
void WindSpeed() {
  windspeed = ((4 * pi * radius * RPM) / 60) / 1000; //Calcula a velocidade do vento em m/s
} //end WindSpeed

//Incrementa contador
void addcount() {
  counter++;
}

double tempoDeIrrigacao(int Ei, float ETc, int Np, float Ne, float qe) {
  float Li = ETc / Ei * 100;
  return (Li * 10000) / (Np * Ne * qe);
}

void logIrrigacao(double tempo, String nomeArq, float ETc, float Etcacumulada) {
  File myFile;
  char dados[65];

  sprintf(dados, "Tempo de Irrigacao = %02d\tETc = %.3f\tETc acumulada = %.3f", tempo, ETc, Etcacumulada);

  myFile = SD.open(nomeArq, FILE_WRITE);
  if (myFile)
  {
    myFile.println(dados);
    myFile.close();
  }
}

void logSituacaoCultura(float TAW, float RAW, String nomeArq, float ET0, float ETc, float Etcacumulada, int dia) {
  File myFile;
  char dados[65];

  sprintf(dados, "Dia = %02d\tTAW = %.3f\tRAW = %.3f\tET0 = %.3f\tETc = %.3f\tETc acumulada = %.3f", dia, TAW, RAW, ET0, ETc, Etcacumulada);


  myFile = SD.open(nomeArq, FILE_WRITE);
  if (myFile)
  {
    myFile.println(dados);
    myFile.close();
  }
}

void datalogger() {

  File myFile;
  char sNomeArq[20];
  char sHoraData[24];

  // Unir tudo em uma unica linha para executar um unico acesso ao cartao SD
  sprintf(sHoraData, "%02d:%02d:%02d %02d/%02d/%4d\t%02d", dt.Hour(), dt.Minute(), dt.Second(), dt.Day(), dt.Month(), dt.Year(), dt.Hour());
  funcaoConstroiStringDados(sLinhaDados, sHoraData);

  // define o nome do Arquivo
  sprintf(sNomeArq, "%02d/a%02d%02d%02d.txt", dt.Month(), dt.Year() % 100, dt.Month(), dt.Day());

  //Gera cabeçalho do arquivo (em hipotese de arquivo)
  if (!SD.exists(sNomeArq))
  {
    myFile = SD.open(sNomeArq, FILE_WRITE);
    delay(500);
    myFile.println("Data\t\t\tHota\tTemperatura (C)\t\tUmidade (%)\t\tPresao (hPa)\t\tVento (m/s)\tRn\tTemp Interna\n");
    myFile.println("\t\t\tUTC\tInst.\tMax.\tMin\tInst.\tMax\tMin.\tInst.\tMax.\tMin.\tVel.\t\tKj/m2\tC\n");
    myFile.close();
    delay(500);
  }

  //Grava os dados no Arquivo
  myFile = SD.open(sNomeArq, FILE_WRITE);
  if (myFile)
  {
    myFile.println(sLinhaDados);
    myFile.close();
  }

  memset(sLinhaDados, 0, sizeof(sLinhaDados));
  sLinhaDados[0] = (char)0; //Vetor de caracteres são terminados por um byte zero, entao zerando apenas o primeiro byte é suficiente para limpar o vetor
  inicializacaoVarClimaticasMaxMin();
}
