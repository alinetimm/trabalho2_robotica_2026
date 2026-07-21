# Relatório Técnico — Trabalho 2: Robô Navegador

**Disciplina:** Tópicos em Sistemas Robóticos / Introdução à Robótica Inteligente — 2026
**Aluna:** Aline Timm
**Link para o vídeo no youtube:** https://youtu.be/z6f-VYwISNY
**Robô:** TurtleBot3 Burger ·
**Ambiente:** ROS 2 Humble + Gazebo Classic



---

## 1. Objetivo

O trabalho pede um nó ROS 2 que conduza o TurtleBot3 Burger de forma autônoma até quatro
estações coloridas — verde (2.20, 2.20), vermelho (2.15, -2.15), azul (-2.16, -2.16) e
laranja (-2.00, 1.20) — retornando à posição inicial (-2.0, 2.0) depois de cada uma, sem
colidir, usando apenas os tópicos `/cmd_vel`, `/odom` e `/scan`.

Este relatório está escrito na ordem em que as coisas realmente aconteceram, incluindo as
duas abordagens que eu tentei antes e abandonei. Faço isso de propósito: foi nos becos sem
saída que eu entendi de verdade por que a solução final é a que é.

---

## 2. Primeira tentativa: reaproveitar um Bug2 que eu já tinha

Eu já havia implementado um navegador Bug2 em um trabalho anterior (no simulador Stage), e a
primeira coisa que fiz foi portá-lo para cá em vez de começar do zero. A ideia parecia sólida:
o Bug2 é um algoritmo *completo* — se existe caminho até o alvo, ele garante chegar — e lida
com paredes côncavas alternando entre ir em linha reta na direção do objetivo (a *m-line*) e
contornar obstáculos quando esbarra em um. Portei a m-line, o seguimento de parede, os gatilhos
de HIT/LEAVE e uma saída antecipada por linha de visada.

Ao rodar no Gazebo, o primeiro problema apareceu logo e foi silencioso. No Stage o `angle_min`
do laser era negativo, então pedir o setor "-25° a 25°" funcionava direto. No TurtleBot3 o
`angle_min` é **0** e são 360 amostras de 1°, de modo que ângulos negativos, ao serem
convertidos em índice, estouravam para baixo e eram "grampeados" no índice 0. Na prática o robô
ficava **cego do lado direito** — os setores da direita liam o mesmo valor da frente. Levei um
tempo até perceber, porque nada quebrava: o robô só se comportava mal. A correção foi reescrever
a indexação para ser **circular** (com "wrap-around": o setor -25°..25° percorre os índices
335..359 e depois 0..25).

O segundo problema foi conceitual e mais interessante: **os alvos são cilindros físicos** de
raio 0,12 m, não pontos abstratos. Com uma tolerância de chegada de 0,15 m o robô nunca
"chegava" — ele batia. E pior: ao se aproximar do alvo, o laser frontal detectava o cilindro
como obstáculo e disparava o seguimento de parede *em volta do próprio objetivo*, orbitando
para sempre. Resolvi isso com uma **janela de aproximação final**: quando a distância ao alvo
cai abaixo de 0,6 m, o gatilho de contorno é desligado e o robô só avança devagar até a
tolerância. Escolhi 0,6 m porque o laser mede a superfície do cilindro, não o centro
(`dist_frontal ≈ dist_centro − 0,12`); para desativar o contorno antes que ele seria disparado
naturalmente eu precisava de `dist > 0,47 m`, e 0,6 dá uma margem confortável sem cortar em cima
da hora.

Mas o fracasso de fundo do Bug2 não foi nenhum desses detalhes — foi a suposição central do
algoritmo. O Bug2 pressupõe que os obstáculos são **fronteiras conectadas** que você contorna
seguindo a parede. Este mapa é o oposto: os obstáculos são segmentos de parede **curtos e
soltos**, mais cilindros isolados, um armário e um cone no meio. Quando o robô começava a
contornar um segmento curto, a parede simplesmente **acabava**; ele perdia a referência, girava
de volta, recolidia, e entrava em ciclo. Os logs mostravam dezenas de eventos de LOOP e STUCK,
141 s só para alcançar o alvo verde — e mesmo assim só por sorte, empurrado por manobras de
escape. Concluí que o Bug2 era a ferramenta errada para este cenário específico.

---

## 3. Segunda tentativa: controle reativo (gap-following / VFH simplificado)

Como o problema do Bug2 era justamente depender de paredes contínuas, troquei por uma
abordagem puramente reativa, inspirada em VFH: dividir o arco frontal (±90°) em setores de 5°,
medir o espaço livre em cada um, e escolher a cada instante o setor que minimiza um custo que
combina *alinhamento ao alvo* com *inverso da distância livre*. Mantive a máquina de estados da
missão, a espera nos alvos, a janela de aproximação e o registro em CSV, e reduzi a navegação a
dois estados: seguir (GO) e recuperar (RECOVER, determinístico).

De novo, dois problemas reais surgiram nos testes:

**Colisões na zona cega do laser.** O LDS-01 tem alcance mínimo de 0,12 m. Quando o robô chega
perto de um obstáculo, o sensor devolve `inf`/`0`/`nan` naquela direção — e eu inicialmente
tratava `inf` como "distância máxima", ou seja, *livre*. O resultado é que uma parede colada no
robô era lida como espaço aberto, e ele avançava: foi assim que ele passou a **empurrar e
arrastar o cone**, chegando a capotar. Corrigi tratando `inf`/`0`/`nan` abaixo do alcance mínimo
como **bloqueado**, e adicionei uma trava dura: se o setor logo à frente (±15°) não está livre,
a velocidade linear vai a zero incondicionalmente, só permitindo girar.

**Mínimos locais.** Este foi o problema que não tinha conserto por ajuste de parâmetro. Como o
controlador só enxerga o scan atual e é *puxado* pelo objetivo, quando o alvo fica atrás de um
bolso de paredes côncavo o robô entra no bolso, todos os setores frontais bloqueiam, o RECOVER
recua e gira, e o GO imediatamente o puxa de volta para dentro do mesmo bolso. Repetindo. No
trecho até o azul cheguei a contar **28 recuperações**; cada perna levava de **4 a 16 minutos**;
a missão completa nunca terminou (estimei 40 a 90 minutos). Isso não cabe nem de longe no vídeo
de 5 minutos, e mais importante, não é navegação — é força bruta.

A lição que tirei aqui foi a mais valiosa do trabalho: **navegação reativa sem memória tinha
batido no teto neste mapa.** Duas abordagens reativas diferentes falharam, cada uma por um
motivo, mas as duas pela mesma razão profunda — um robô que só reage ao instante presente e é
atraído pelo objetivo não tem como *saber* que está numa armadilha, porque perceber a armadilha
exige lembrar por onde já se passou.

---

## 4. Abordagem final: mapa de ocupação + planejamento wavefront + pure-pursuit

A decisão foi dar memória ao robô. É importante frisar que isso **continua respeitando a
restrição** de usar só `/cmd_vel`, `/odom` e `/scan`: eu construo o mapa por conta própria a
partir desses sensores, sem Nav2, sem AMCL e sem mapa externo pré-carregado.

O funcionamento é o seguinte. Mantenho uma **grade de ocupação** fixa no mundo, cobrindo a
arena (x, y em [-2.5, 2.5]) com resolução de 0,05 m — cerca de 100×100 células, trivial de
processar num espaço tão pequeno. A cada leitura do laser, para cada raio com retorno válido
(descartando `inf`/`0`/`nan`), projeto o ponto de impacto no referencial do mundo usando a pose
da odometria e marco aquela célula como ocupada; o mapa é acumulativo. Em seguida **inflo** as
células ocupadas por um raio de segurança (raio do robô + margem, ~4 células) — é essa inflação
que elimina as colisões, porque o planejador simplesmente nunca traça uma rota que passe perto
demais de um obstáculo.

Sobre esse mapa roda um **planejador wavefront (BFS)**, da célula do robô até a do objetivo,
recalculado a alguns Hz e sempre que a rota é bloqueada por algo recém-visto. Como o alvo é um
cilindro, a célula do centro cai dentro da região inflada e é inalcançável; contornei isso
planejando para a célula livre mais próxima do centro, na distância da tolerância, e entregando
daí em diante para a janela de aproximação que eu já tinha. Para a base, que não tem obstáculo
em cima, o planejamento vai direto.

O caminho é seguido por um controlador **pure-pursuit** simples: escolho um ponto do caminho a
~0,3 m à frente, calculo o erro angular até ele e aplico controle proporcional na rotação,
saturando nos limites do Burger, com a velocidade linear zerada quando o erro angular é grande
(gira parado antes de avançar). Por cima de tudo isso permanece a trava de segurança reativa da
etapa anterior, agora como última linha de defesa e não como estratégia principal.

### Respondendo diretamente aos itens do enunciado

**Estratégia de navegação.** Mapeamento de ocupação incremental a partir de laser + odometria,
com planejamento global sobre o que já foi visto (wavefront/BFS) e replanejamento periódico. Uma
máquina de estados de missão sequencia alvo → base → próximo alvo, com espera não-bloqueante nas
chegadas. A vantagem decisiva sobre o reativo é que o planejador tem visão do mapa acumulado, o
que **elimina os mínimos locais** que travavam as abordagens anteriores.

**Método de controle.** Pure-pursuit com um ponto-alvo à frente (*lookahead*) e controle
proporcional sobre o erro de orientação, com saturação nos limites físicos do Burger
(0,22 m/s e 2,84 rad/s, usados com folga) e um portão que alinha o robô parado antes de
avançar em curvas fechadas. A posição vem exclusivamente de `/odom`.

**Estratégia de desvio de obstáculos.** Duas camadas. A principal é a **inflação** da grade de
ocupação: como as células perto de qualquer obstáculo detectado por `/scan` são marcadas como
proibidas, o planejador já entrega rotas com folga e o desvio acontece por construção. A
secundária é uma **parada dura reativa**: se o setor frontal estiver bloqueado (tratando as
leituras cegas de perto como obstáculo), a velocidade linear é anulada, evitando colisão mesmo
que o mapa esteja momentaneamente desatualizado.

---

## 5. Resultados

A mudança de abordagem teve efeito grande e mensurável:

| | Reativo (VFH) | Mapa + wavefront |
|---|---|---|
| Tempo por perna | 4–16 min | 20–33 s |
| Colisões | sim (2 corrigidas) | zero |
| Travamentos | dezenas de recuperações | zero (nesta execução) |
| Missão completa | não concluiu (~40–90 min est.) | **239,5 s (~4 min)** |

Na execução completa registrada, a grade foi mapeada por inteiro (visível no PNG de
trajetória), o robô fez as quatro idas-e-voltas de forma limpa, sem replanejamentos travados e
sem colisões. Os erros de chegada finais por alvo, extraídos do CSV, foram:
verde [__ m], vermelho [__ m], azul [__ m], laranja [__ m] *(preencher a partir do
`mission_*.csv` da corrida final)*.

*(Figuras: trajetória da missão completa colorida por estado, com a grade inflada ao fundo e os
quatro alvos e a base marcados. Anexar os PNGs `verde+home`, `azul+home` e `missao_completa`.)*

---

## 6. Principais dificuldades encontradas

- **Convenção do laser entre simuladores.** O `angle_min = 0` do TurtleBot3 (contra o valor
  negativo do Stage) fazia os setores da direita serem lidos como frontais, deixando o robô cego
  de um lado sem gerar nenhum erro visível. Exigiu indexação circular.
- **Alvo que também é obstáculo.** Os cilindros de raio 0,12 m tornavam a tolerância ingênua
  inatingível e faziam o robô contornar o próprio objetivo. Resolvido com a janela de
  aproximação final.
- **Zona cega do laser.** Com alcance mínimo de 0,12 m e leituras `inf`/`0`/`nan` quando algo
  está colado, tratar essas leituras como "livre" causava colisões (o robô arrastava o cone).
  Passaram a contar como obstáculo.
- **Mínimos locais.** O grande motivo do fracasso do reativo: bolsos côncavos aprisionavam um
  controlador que só reage ao instante presente. Só o mapa + planejador resolveu de fato.
- **Obstáculos desconexos.** Segmentos de parede curtos quebravam a premissa do Bug2 de
  fronteiras contínuas.
- **Base encravada.** A posição inicial (-2, 2) fica num canto protegido por paredes internas,
  o que tornava o retorno especialmente difícil para as abordagens reativas.

---

## 7. Limitações observadas

A abordagem final resolve o problema central, mas tem limites que observei nos testes e prefiro
declarar com honestidade:

- **O mapa de ocupação não "esquece".** Como é acumulativo, uma leitura espúria do laser marca
  uma célula como ocupada permanentemente. Numa das execuções do trajeto até o azul, isso criou
  um bloqueio falso que travou o planejamento momentaneamente. Reproduzi uma vez, instrumentei o
  código para registrar o motivo exato e salvar o mapa quando isso ocorre, e alarguei a busca por
  célula de aproximação como mitigação — mas **não considero 100% resolvido**. O importante é que
  o travamento nunca é permanente: o limite de tentativas dispara uma manobra de ré e giro
  determinística, sem loop infinito.
- **Deriva de odometria.** A posição vem só de `/odom`, sem correção; em trajetos longos o erro
  acumulado desloca ligeiramente o mapa. Na escala desta arena (~4,85 m) o efeito é pequeno, mas
  existe.
- **Parâmetros ajustados para este ambiente.** Resolução da grade, raio de inflação e
  *lookahead* foram calibrados para este mapa e este robô; não são universais.
- **Sem obstáculos dinâmicos.** O mapa acumulativo pressupõe um ambiente estático — não trata bem
  objetos que se movem (o cone, se empurrado, permaneceria marcado na posição antiga).
- **A camada reativa é um último recurso**, não a estratégia; se o mapa estivesse muito
  desatualizado, ela apenas evitaria a colisão, sem garantir progresso.

---

## 8. Conclusão

O caminho que percorri — Bug2, depois reativo, depois mapa de ocupação — me ensinou que o
gargalo em navegação raramente é o ajuste fino: é a adequação entre as suposições do método e o
ambiente. As duas primeiras abordagens não falharam por parâmetros ruins, mas porque suas
premissas (fronteiras contínuas; suficiência da reação instantânea) não valiam neste mapa
específico. Dar ao robô uma memória do que já viu e um planejador que enxerga essa memória
transformou uma missão que se arrastava por dezenas de minutos, com colisões e travamentos, numa
execução limpa de cerca de quatro minutos.