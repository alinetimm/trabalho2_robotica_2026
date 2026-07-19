# Relatório Técnico — Navegação Autônoma do TurtleBot3 Burger

## 1. Estratégia de navegação utilizada

A navegação é feita com o algoritmo **Bug2**, escolhido por não exigir mapa prévio do
ambiente: o robô só usa a pose estimada (odometria) e o LIDAR (`/scan`), que é
exatamente a informação disponível no enunciado.

O Bug2 alterna entre dois modos:

- **GO_TO_GOAL** — segue em linha reta na direção do alvo enquanto o caminho está livre.
- **WALL_FOLLOW** — quando encontra um obstáculo, contorna a sua borda até poder
  retomar o GO_TO_GOAL.

O critério clássico do Bug2 usa uma **m-line**: a reta entre o ponto onde o contorno
começou (`ml_sx, ml_sy`) e o alvo atual. O robô sai do contorno quando volta a cruzar
essa reta (tolerância `bug2.mline_tol = 0.30 m`) e está mais perto do alvo do que
estava no momento em que bateu no obstáculo (`hit_dist`). Além do critério de m-line,
foi adicionado um critério extra de **linha de visada (LOS)**: se o setor frontal na
direção do alvo está livre por uma distância maior que a distância restante até ele,
o robô sai do contorno imediatamente, sem esperar cruzar a m-line — isso evita
contornos desnecessariamente longos em cantos convexos.

Acima do Bug2 existe uma **camada de missão**, definida em
`ros_ws/src/robot_navigation/config/mission.yaml` e carregada via `declare_parameter`
(nenhuma coordenada fica hard-coded no código): a lista de waypoints é montada
intercalando cada alvo com a posição inicial (`home`), gerando a sequência

```
verde → home → vermelho → home → azul → home → laranja → home
```

Cada waypoint tem nome, posição e tolerância de chegada próprias (`home.tol = 0.15 m`,
`targets.*.tol = 0.35 m`). Ao chegar a um waypoint, o robô publica velocidade zero,
registra o evento (posição, erro e tempo decorrido) e permanece parado por
`wait_time = 2 s` (estado `WAITING`, sem bloquear o loop de controle) antes de seguir
para o próximo. Ao final, um resumo é logado: tempo total, tempo por waypoint e
número de contornos executados.

## 2. Método de controle adotado

O controle é **reativo baseado em regras**, não um PID contínuo: a cada leitura o
robô calcula o erro angular até o alvo (`ga_r`) e decide o comportamento por faixas:

- erro angular > 45° → gira no lugar (`linear.x = 0`) até se alinhar melhor;
- erro angular entre 15° e 45° → avança devagar enquanto corrige o rumo;
- erro angular < 15° → avança com `linear.x = LIN * speed_factor` e correção
  proporcional ao erro (`angular.z = ga_r * ganho`).

`speed_factor` reduz a velocidade linear conforme a distância livre à frente
diminui, funcionando como uma frenagem antecipada perto de obstáculos.

Os ganhos (`bug2.lin = 0.18 m/s`, `bug2.ang = 0.9 rad/s`, `bug2.wall_dist = 0.40 m`)
são parâmetros ROS carregados do YAML, dimensionados a partir dos limites físicos do
Burger (`v_max ≈ 0.22 m/s`, `w_max ≈ 2.84 rad/s`) com margem de segurança — não usamos
o limite máximo do robô para deixar folga de resposta ao controlador.

Para o trecho final de aproximação a um alvo (ver seção 3), existe um terceiro modo
de controle, mais conservador: apenas a lei de GO_TO_GOAL, com velocidade linear
limitada a `approach.max_speed = 0.08 m/s`.

## 3. Estratégia de desvio de obstáculos

O `/scan` (360 amostras, 1°, `angle_min = 0`) é dividido em 5 setores angulares —
frontal (±25°), diagonais frontais (15°–60° e -60°–-15°) e laterais (60°–120° e
-120°–-60°) — através de indexação circular (`idx % 360`), necessária porque o
Burger reporta ângulo mínimo 0 (não -180°), então setores que cruzam 0°/360°
precisam de *wrap-around* explícito no cálculo do menor alcance do setor.

Quando o setor frontal fica abaixo de `D_OBS = 0.35 m`, o robô entra em
`WALL_FOLLOW`. O lado do contorno é escolhido comparando o espaço livre à esquerda e
à direita (com bônus para o lado mais alinhado ao alvo), e mantido por histerese
(30 ciclos) para não trocar de lado repetidamente perto de quinas. Durante o
contorno, o controlador mantém uma distância alvo da parede (`WALL_DIST = 0.40 m`)
por controle proporcional ao erro lateral, com parada total abaixo de 0.30 m.

Dois mecanismos evitam loops infinitos ao redor do mesmo obstáculo: **stuck**
(sem progresso em direção ao alvo após 250 ciclos) e **back_to_hit** (retornou perto
do ponto onde bateu no obstáculo após 150 ciclos) — ambos invertem o lado do
contorno.

**Caso especial — os 4 alvos são cilindros físicos** (raio 0.12 m, robô com raio
~0.105 m), então o Bug2 puro trataria o próprio alvo como obstáculo e ficaria
contornando-o para sempre ao chegar perto. Para isso existe uma **janela de
aproximação final** (`approach.window_dist = 0.6 m`): dentro dela, o gatilho de HIT
fica desativado e qualquer `WALL_FOLLOW` em andamento é interrompido imediatamente,
trocando para uma aproximação direta e lenta. Uma exceção de segurança
(`approach.safety_fm = 0.18 m`) para o avanço e gira no lugar caso algo *além* do
alvo esperado apareça perto demais — o valor foi escolhido abaixo da distância de
laser esperada quando o robô está exatamente na tolerância de chegada
(`0.35 m − 0.12 m ≈ 0.23 m`), para não disparar falsamente durante uma aproximação
normal.

## 4. Principais dificuldades encontradas

- **Portar o Bug2 de outro simulador (Stage) para o TurtleBot3/Gazebo**: o código
  original assumia `angle_min = -180°`; o Burger usa `angle_min = 0`, exigindo
  reescrever a indexação de setores com aritmética circular — validado com testes
  sintéticos colocando marcadores em índices específicos do laser e conferindo que
  cada setor lê o intervalo certo, inclusive atravessando o 0°/360°.
- **Alvos como obstáculos físicos**: diferente do ambiente original, aqui os alvos
  colidem fisicamente com o robô. Isso só ficou evidente ao analisar a geometria
  (raio do alvo somado ao raio do robô), e exigiu desenhar a janela de aproximação
  final como um modo de controle à parte, não apenas ajustar uma tolerância.
- **Log de missão sem bloquear o `timer` de controle**: a espera de 2 s em cada
  waypoint não podia usar `time.sleep()` dentro do callback (travaria as assinaturas
  de `/odom` e `/scan`), então foi implementada como estado `WAITING` comparando
  `self.get_clock().now()` a cada ciclo.
- **Ambiente de build**: pastas `.orig` remanescentes de uma conversão anterior para
  git submodules causavam conflito de nomes de pacote no `colcon build`
  (`turtlebot3_msgs`, `turtlebot3_gazebo` duplicados); depois disso, o submodule de
  `turtlebot3_simulations` estava fixado num commit que migrou para o Gazebo novo
  (`gz-sim`), incompatível com a imagem Docker (Gazebo Classic) e com os plugins do
  `.world`. Ambos foram corrigidos, mas a compilação do `turtlebot3_gazebo` ainda não
  terminou de ponta a ponta — o container caiu novamente no meio da compilação.

## 5. Limitações observadas durante os testes

**Importante:** até o momento, a validação foi feita **isoladamente**, alimentando o
nó com mensagens sintéticas de `Odometry` e `LaserScan` fora do Gazebo — não houve
ainda uma execução completa da missão no simulador real. Isso cobre a lógica de
estados, a indexação do laser e a janela de aproximação, mas não substitui teste com
física real, ruído de sensor e o robô de verdade. Essa validação em Gazebo ainda está
pendente e deve ser feita antes da entrega final.

Limitações do próprio algoritmo, independentes do ambiente de simulação:

- **Sem mapa global**: por ser puramente reativo, o Bug2 pode gerar contornos longos
  se a geometria dos obstáculos for desfavorável à escolha de lado — os mecanismos de
  histerese, `stuck` e `back_to_hit` reduzem mas não eliminam esse risco.
- **Ganhos fixos**: `D_OBS`, `WALL_DIST` e os demais parâmetros de controle foram
  dimensionados por raciocínio geométrico (raio do robô, raio dos alvos, distâncias
  das paredes), não por ajuste empírico contra ruído real do LIDAR — podem precisar
  de retoque após os primeiros testes em Gazebo.
- **Odometria pura**: não há nenhuma correção de deriva (sem filtro, sem referência
  externa). Numa missão longa (4 idas e voltas à base) o erro acumulado de odometria
  pode crescer o suficiente para afetar a precisão de chegada, especialmente perto da
  tolerância mais apertada da home (0.15 m).
- **Limiar de segurança da janela final** (0.18 m) foi calculado a partir da geometria
  ideal (raio do alvo, offset do LIDAR); ainda não foi confirmado contra leituras
  reais com ruído do sensor simulado.
