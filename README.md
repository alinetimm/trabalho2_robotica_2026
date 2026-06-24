# Trabalho Prático - Navegação de Robôs Móveis com ROS 2 e Gazebo

## Objetivo

Desenvolver um nó ROS 2 capaz de navegar autonomamente com o TurtleBot3 Burger em um ambiente simulado no Gazebo.

O robô deve localizar e alcançar uma sequência de alvos definidos no ambiente, evitando colisões com obstáculos e retornando à posição inicial após cada visita.

---

## Ambiente Utilizado

* ROS 2 Humble
* Gazebo Classic
* TurtleBot3 Burger
* Docker

Todo o ambiente de desenvolvimento é executado dentro de um container Docker, garantindo que todos os alunos utilizem a mesma configuração.

---

## Estrutura do Projeto

```text
.
├── docker/
│   └── Dockerfile
├── launch/
│   └── turtlebot3_dqn_stage5.launch.py
├── world/
│   └── turtlebot3_dqn_stage5.world
├── ros_ws/
│   └── src/
├── setup.sh
├── run.sh
└── control_terminal.sh
```

---

## Configuração Inicial

Execute apenas uma vez:

```bash
./setup.sh
```

O script realiza automaticamente:

* Download dos pacotes TurtleBot3 necessários;
* Configuração do workspace ROS 2;
* Instalação das dependências;
* Construção da imagem Docker utilizada na disciplina.

---

## Executando o Ambiente

Inicie o container:

```bash
./run.sh
```

Compilar as bibliotecas:

```bash
colcon build
```

Configurar o ambiente:

```bash
source install/setup.bash
```

---

## Iniciando o Simulador

Dentro do container execute:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage5.launch.py
```

Uma janela do Gazebo deverá ser aberta contendo o ambiente utilizado para o trabalho.

---

## Criando o Seu Pacote

Abra um novo terminal e acesse o container (igual a parte do `./run.sh`):

```bash
./control_terminal.sh
```

Dentro do container, navegue até o workspace:

```bash
cd /ros_ws/src
```

Crie um novo pacote ROS 2:

### Python

```bash
ros2 pkg create \
    --build-type ament_python \
    robot_navigation
```

### C++

```bash
ros2 pkg create \
    --build-type ament_cmake \
    robot_navigation
```

Após a criação do pacote, compile o workspace:

```bash
cd /ros_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install
```

Carregue o ambiente:

```bash
source install/setup.bash
```

---

## Tópicos Disponíveis

### Controle do Robô

```text
/cmd_vel
```

Tipo:

```text
geometry_msgs/msg/Twist
```

Responsável por receber velocidades linear e angular.

---

### Odometria

```text
/odom
```

Tipo:

```text
nav_msgs/msg/Odometry
```

Fornece a estimativa da posição e orientação do robô.

---

### Sensor Laser

```text
/scan
```

Tipo:

```text
sensor_msgs/msg/LaserScan
```

Fornece medições de distância dos obstáculos ao redor do robô.

---

## Testando o Movimento

Mover para frente:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.2}, angular: {z: 0.0}}" \
-r 10
```

Parar o robô:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

---

## Tarefa

Desenvolver um nó ROS 2 capaz de:

1. Navegar até todos os alvos definidos no ambiente;
2. Evitar colisões com obstáculos;
3. Retornar à posição inicial após cada visita;
4. Executar toda a sequência de forma autônoma.

A estratégia de navegação é livre.

Podem ser utilizados:

* Controle reativo;
* Máquina de estados;
* Outras abordagens estudadas durante a disciplina.

---

## Entrega

A entrega deve conter:

### Código-fonte

Projeto ROS 2 completo.

### Vídeo

Vídeo curto demonstrando:

* Inicialização do sistema;
* Navegação até todos os alvos;
* Retorno à posição inicial.

### Relatório

Descrever:

* Estratégia adotada;
* Método de navegação;
* Método de desvio de obstáculos;
* Principais dificuldades encontradas;
* Limitações observadas durante os testes.

---

## Critérios de Avaliação

* Funcionamento correto do sistema;
* Organização do código;
* Qualidade da solução proposta;
* Capacidade de evitar colisões;
* Clareza do relatório e do vídeo.

