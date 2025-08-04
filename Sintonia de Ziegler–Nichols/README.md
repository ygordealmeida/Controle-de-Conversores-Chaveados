# Controle de Conversores DC-DC Buck usando a sintonia Ziegler Nichols

#### Este repositório documenta o processo completo de projeto, um controlador P/PI/PID para um conversor DC-DC, utilizando o método de sintonia de Ziegler-Nichols. 
---

## Etapas do Projeto


### Visão Geral

A ideia por trás da metodologia a ser apresentada aqui é a de projetar um controlador de forma simples e rápida, isto é, evitando cálculos complexos, como a elaboração de um modelo de pequenos sinais do conversor, e que atenda os requisitos mínimos de controle. Assim torna-se necessário que o projetista conheça apenas o dimensionamento dos elementos do conversor e posso realizar simulações sobre ele.




### 1. Simulação do Conversor

-Inicialmente é necessário simular o convesor em algum software, nesse caso vamos utilizar o PSIM, pois ele permite a utlização de blocos de programação em C que serão utéis futuramente. Nesse caso, vamos simular um conversor Buck, com tensão de entrada de $50V$ e buscando uma tensão de saída de $25V$, nesse caso pela fórmula do ganho estático desse convesor o _Duty Cycle_ deve ser de $0.5$. Logicamente, vamos projetar um controlador para que a saída siga a referência mesmo que ocorram perturbações no circuito.

<img width="1544" height="670" alt="image" src="https://github.com/user-attachments/assets/3260dda9-539b-4f49-aaac-5fd6de66122c" />

#### Sintonia de Ziegler-Nichols _(Ultimate Gain)_

- Inicialmente, deve ser implementado um controle proporcional puro, isto é um ganho na malha de realimentação.

<img width="586" height="222" alt="image" src="https://github.com/user-attachments/assets/1c48935e-5827-4bef-80a2-85b558ec102e" />

- A constante de ganho proporcional (K) é aumentada até que o sistema entre em **estabilidade marginal**, ou seja, oscila com amplitude constante e sem crescimento, este estado está entre a estabilidade e a instabilidade.
- Para o caso do conversor Buck não é possível alcançar a instabilidade, pois a tensão de saída é no maximo igual a de entrada, assim o ponto de estabilidade marginal é o menor ganho que causa oscilações controladas.
- Isso fornece os parâmetros críticos:
  - **Ku**: ganho no ponto de oscilação marginal.
  - **Pu**: período da oscilação marginal (em segundos)

<img width="1336" height="776" alt="image" src="https://github.com/user-attachments/assets/3566d68d-5427-403c-91b4-6e77ee8b2c6e" />

<img width="1895" height="850" alt="image" src="https://github.com/user-attachments/assets/30124bb0-f522-42df-ad1d-74336e9af3a7" />

Aqui encontramos:
```
Ku = 0.37 
Pu = 6.05487e-04
```
#### 1.2. Cálculo dos Parâmetros PI

Em posse de Ku e Pu é possível cálcular valores de P/PI/PID a partir da tabela a seguir:

<img width="1624" height="803" alt="image" src="https://github.com/user-attachments/assets/b6f4e570-0224-481c-bd5d-85f784c78a89" />

Utilizando a forma padrão do método de Ziegler-Nichols para controladores PI:

```
Kp = 0.45 * Ku = 0.1665
Ti = Pu / 1.2 =  5.045725e-04
```

No PSIM, o bloco PI usa a forma:

$$
G(s) = k \cdot \frac{1 + sT}{sT}
$$

Logo, configuramos:

- **Gain (k)**: valor de Kp calculado.
- **Time Constant (T)**: igual a Ti.

  <img width="445" height="285" alt="image" src="https://github.com/user-attachments/assets/febd6a46-6d78-47e6-94b9-551b4b058065" />



  É importante estabelecer os limites inferior e superior do seu bloco PI, de forma que se adeque ao _Duty Cycle_

<img width="1236" height="736" alt="image" src="https://github.com/user-attachments/assets/a2037865-b593-457e-b565-a52b842c0e4b" />


<img width="1900" height="853" alt="image" src="https://github.com/user-attachments/assets/c2d191a2-133e-4aae-aacb-046b4861d302" />


### 2. Anti-Windup

O Anti-windup é um mecanismo que impede que o integrador continue acumulando erro quando a saída do controlador está saturada. Isso é essencial para evitar instabilidades e recuperação lenta.

---

## Controle Digital (via Microcontrolador)

Nessa etapa, vamos abordar como transferir o controlador do meio analogico para o digital, para que possa ser implementado via microcontrolador.

### 1. Discretização

No Matlab vamos discretizar a equação no dóminio de Laplace que obtemos do bloco PI do PSIM, para o domínio de Z, e encontrar os coeficientes da equação de diferenças
Para o controlador PI a função em Z e a equação de diferenças seguem o formato a seguir (para mais detalhes consulte o anexo):

$$
G_c(z) = \frac{b_0 + b_1 z^{-1}}{1 + a_1 z^{-1}}
$$

$$
u[n] = -a_1 u[n-1] + b_0 e[n] + b_1 e[n-1]
$$

No Matlab usamos a transformação bilinear para discretizar a função, com $Ts = 10us$ que é período de amostragem usado no nosso controlador, em seguida coletamos os coeficientes da função discretizada

Kp = 0.1665;
Ti = 5.045725e-4;

num = Kp * [Ti 1];   % Numerador: Kp*(Ti*s + 1)
den = [Ti 0];         % Denominador: Ti*s

G_s = tf(num, den)

Ts = 1e-5;  % 10 us

G_z = c2d(G_s, Ts, 'tustin')

[num_d, den_d] = tfdata(G_z, 'v')

% Coeficientes extraídos:
b0 = num_d(1)
b1 = num_d(2)
a1 = den_d(2) 


### 2. Implementação em Código
Por fim implementamos a equação de diferença no _Simplified C block_, do PSIM, que é similar a um microcontrolador. Ainda em código, fazemos uma onda triangular de 5Khz para comparar com a saída do PI e gerar o sinal PWM de controle. Em nossa simulação o passo de cálculo é de 1us, isso deve ser levado em conta na definição da onda triangular e no operação de controle que foi feita para 10us. A seguir o código utilizado:

static double u_n, e_n;
static double u_n_1 =0;
static double e_n_1 =0;
static double tempo=0;
const int V_ref = 25;

static double triangular = 0;
static double PI=0;
static int a=1;


double controle(double error){



tempo=0;
e_n = error;


u_n = u_n_1 + 0.16814962*e_n - 0.16485038*e_n_1;




u_n_1 = u_n;
e_n_1 = e_n;

return (u_n);
}

double onda_triangular (void){

triangular = triangular + 2*0.01*a;

if (triangular >= 1){

a = -1;}


if (triangular <= -1){

a=1;
}

}

onda_triangular();
tempo = tempo + delt;
if(tempo >= 0.000010){

PI = controle(V_ref-x1);

}

if(PI>triangular){
y1 = 1;
}

else if(PI < triangular){
y1= 0;
}


## Observações Importantes

- O conversor buck é naturalmente estável e limitado em tensão, por isso não tende a explodir, mesmo com ganhos elevados.
- Pequenas oscilações em regime permanente são aceitáveis.
- A seleção de `Ts` é fundamental: geralmente é 10 a 20 vezes mais rápido que a dinâmica dominante do sistema.

---

## Ferramentas Utilizadas

- **PSIM** para simulações analógicas.
- **MATLAB** para análise de controle e discretização.
- **C** (ou similar) para implementação embarcada no microcontrolador.

---



## Licença

MIT License

---

Este projeto documenta uma implementação educativa de controle PI para conversores chaveados. Para o caso de dúvidas ygor.pereira@ee.ufcg.edu.br


