# Bola no swing de golfe

Executar na pasta do projeto:

```powershell
python src/main.py --mode physics
```

A bola branca está sobre um suporte (tee). O taco atinge-a durante o swing;
ela desloca-se, cai e rola no chão. Backspace repõe a bola no suporte e
reinicia o movimento. O marcador verde continua a indicar o alvo do taco.
No modo `kinematics`, a bola fica parada: esse modo apenas altera as poses
do jogador e não integra a dinâmica nem resolve impactos.

## Como foi feito

1. **Esfera independente.** `assets/ball.xml`, incluído por `assets/main.xml`,
   acrescenta um corpo `bola` diretamente ao mundo. A geometria é uma esfera
   branca com raio de 0,02135 m e massa de 0,04593 kg.

   ```xml
   <body name="bola" pos="0.115 0 0.146">
     <freejoint name="bola_livre"/>
     <geom name="bola" type="sphere" size="0.02135" mass="0.04593"
           rgba="1 1 1 1" contype="0" conaffinity="0"/>
   </body>
   ```

   A `freejoint` permite três translações e três rotações. A bola não tem motor;
   o MuJoCo calcula o seu movimento pela gravidade e pelas forças de contacto.

2. **Posição alcançável.** Medi a trajetória física da cabeça do taco. Na
   passagem inferior, o seu centro fica a aproximadamente 0,146 m do chão.
   A bola está em `(0.115, 0, 0.146)` m, mais à frente nessa passagem. Um
   cilindro de raio 0,008 m suporta-a, com o topo a
   `0.146 - 0.02135 = 0.12465` m. Este suporte é alto para adaptar a bola à
   geometria atual do exercício; a trajetória existente não foi alterada.

3. **Contactos explícitos.** O bloco `<contact>` declara os pares
   `cabeca_taco/bola`, `floor/bola` e `tee/bola`. As máscaras nulas da bola e
   do suporte desativam pares automáticos; os pares explícitos continuam
   ativos. Assim, o marcador verde não bate na bola. `condim="6"` inclui
   contacto normal e atrito de deslizamento, torção e rolamento. O taco não
   colide com o suporte, uma simplificação deste modelo.

4. **Bola livre durante a simulação.** Antes, `main.py` bloqueava todas as
   juntas que não eram comandadas pelo controlador. Isso também bloquearia
   `bola_livre`. O novo método `get_locked_humanoid_joints`, em `src/env.py`,
   percorre a árvore de corpos e seleciona apenas juntas não comandadas do
   jogador, descendentes de `torso`. A bola permanece independente.

5. **Impacto e reinício.** `mj_step` resolve a colisão e transmite força do
   taco à bola. Não se atribui manualmente uma velocidade de lançamento.
   O `mj_resetData` já usado pelo projeto também repõe a posição e a velocidade
   da nova bola quando se carrega em Backspace.

## Verificação

```powershell
python -m unittest discover -s tests -p test_ball.py -v
```

Os quatro testes verificam a junta livre, o suporte antes da pancada, o
contacto com força positiva, a deslocação e chegada ao chão, o reinício e
a queda livre longe do suporte.

Com MuJoCo 3.13.0 e o passo atual de 0,005 s, a simulação de seis segundos
registou o primeiro contacto aos 1,86 s e a bola terminou aproximadamente em
`(0.029, 1.761, 0.02135)` m, sem avisos do motor físico. O contacto também
ocorreu com passos de 0,0025 e 0,00125 s. A distância final varia com o passo
temporal; os parâmetros são demonstrativos e não estão calibrados para
prever a distância de uma tacada real.
