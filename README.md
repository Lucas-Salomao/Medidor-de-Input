# Dispositivo portátil de medição de input

Este dispositivo é capaz de medir o tempo entre entre 2 pulsos de um sensor e converter esse tempo em uma tensão de 0-5V, dividido em 2 canais, um para segundos inteiros e outro para milisegundos.

## Características

- Tensão de alimentação: 10-30 VDC
- Display LCD de 2 linhas e 16 colunas para exibição
- Memória EEPROM para armazenar configuração e dados de calibração.
- 1 porta de comunicação serial-USB
- Encoder rotativo para parametrização
- Acurácia de ± 1LSB
- 12 bits de resolução

## Diagrama Esquemático

<img src="/imgs/diagrama_bb.png">

## Pinout Arduino Nano

<img src="/imgs/Mapa-de-Pinos-pinout-Arduino-NANO-Original.png">

## Procedimentos para Compilação e Gravação

O projeto foi desenvolvido no Visual Studio Code, utilizando a extensão [PlatformIO](https://platformio.org/).

É necessário ter o [Arduino IDE Legacy](https://www.arduino.cc/en/software) instalado para poder compilar e gravar a aplicação.

Fazer o download deste repositório, descompactar a pasta e abrir a pasta pelo Visual Studio Code. Assim toda a estrutura do projeto será reconhecida.

<img src="/imgs/estrutura.png">

Para compilar o projeto basta utilizar o seguinte atalho no teclado: CTRL+ALT+B. É possível também utilizar o atalho clicando no ícone que se encontra no rodapé do Visual Studio Code.

<img src="/imgs/compilar.png">

Para fazer o upload do programa para o dispositivo, com o cabo USB inserido no dispositivo e no computador, utilize o seguinte atalho no teclado: CTRL+ALT+U. É possível também utilizar o atalho clicando no ícone que se encontra no rodapé do Visual Studio Code.

<img src="/imgs/upload.png">

Todas as bibliotecas utilizadas são listadas no arquivo [platfomio.ini](/platformio.ini) e são instaladas assim que a estrutura do projeto é aberto no Visual Studio Code. Para utilizar o Arduino IDE será necessário instalar manualmente cada biblioteca.

## Modo de Funcionamento

Ao energizar o equipamento está pronto para operação, apresentando a seguinte tela:

<img src="/imgs/tela1.PNG">

Nesta tela temos as 2 opções disponíveis, Monitoramento e Configuração.

O encoder rotativo é utilizado para a navegação no menu.

Rotacionando no sentido horário, os valores são incrementados e o menu é deslocado para baixo.

Rotacionando no sentido anti-horário, os valores são decrementados e o menu é deslocado para cima.

O encoder rotativo também apresenta um botão. Quando pressionado o centro do encoder, esse botão é acionado. O Botão apresenta 3 funções:

- Pressionando 1 vez o comando ENTER é enviado.
- Prssionando 2 vezes o comando VOLTAR é enviado e o menu volta um nível acima.
- Pressionando 3 vezes o comando RESET é enviado e o dispositivo é reiniciado.
- Pressionando 1 vez e segurando pelo menos 500ms, o comando APAGAR é enviado e o valor atual selecionado no display é apagado.

No menu Monitoramento podemos encontrar os valores de tempo medidos e os respectivos valores de tensão na saída, calculados.

<img src="/imgs/telamonitor1.PNG">

<img src="/imgs/telamonitor2.PNG">

No menu Configuração podemos configurar os seguintes parâmetros:

- Quantidades de Voltas
- Tempo Máximo de Amostragem
- Precisão da Saída Analógica
- Habilitar Rotina de Teste
- Tempo da Rotina de Teste
- Apagar Memória EEPROM

<img src="/imgs/telaconfig1.PNG">

<img src="/imgs/telaconfig2.PNG">

<img src="/imgs/telaconfig3.PNG">

<img src="/imgs/telaconfig4.PNG">

## Licença

[MIT](https://choosealicense.com/licenses/mit/)

## Suporte

Para suporte, mande um e-mail para lucastadeusalomao@gmail.com.