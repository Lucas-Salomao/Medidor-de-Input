# Dispositivo portátil de medição de input

Este dispositivo é capaz de medir o tempo entre entre 2 pulsos de um sensor e converter esse tempo em uma tensão de 0-5V, dividido em 2 canais, um para segundos inteiros e outro para milisegundos.

## Características

- Tensão de alimentação: 24 VDC
- Display LCD de 2 linhas e 16 colunas para exibição
- Memória EEPROM para armazenar configuração e dados de calibração.
- 1 porta de comunicação serial-USB
- Encoder rotativo para parametrização

## Diagrama Esquemático

<img src="/imgs/diagrama_bb.png">

## Procedimentos para Compilação e Gravação

O projeto foi desenvolvido no Visual Studio Code, utilizando a extensão [PlatformIO](https://platformio.org/).

É necessário ter o [Arduino IDE Legacy](https://www.arduino.cc/en/software) instalado para poder compilar e gravar a aplicação.

Fazer o download deste repositório, descompactar a pasta e abrir a pasta pelo Visual Studio Code. Assim toda a estrutura do projeto será reconhecida.

<img src="/imgs/estrutura.png">

Para compilar o projeto basta utilizar o seguinte atalho no teclado: CTRL+ALT+B. É possível também utilizar o atalho clicando no ícone que se encontra no rodapé do Visual Studio Code.

<img src="/imgs/compilar.png">

Para fazer o upload do programa para o dispositivo, com o cabo USB inserido no dispositivo e no computador, utilize o seguinte atalho no teclado: CTRL+ALT+U. É possível também utilizar o atalho clicando no ícone que se encontra no rodapé do Visual Studio Code.

<img src="/imgs/upload.png">

## Licença

[MIT](https://choosealicense.com/licenses/mit/)

## Suporte

Para suporte, mande um email para lucastadeusalomao@gmail.com.