# Controle de Conversores DC-DC usando a sintonia Ziegler Nichols

#### Este repositório documenta o processo completo de projeto, um controlador P/PI/PID para um conversor DC-DC, utilizando o método de sintonia de Ziegler-Nichols. 


### Sumário

- [1. Visão Geral](#1-visão-geral)
- [2. Simulação do Conversor](#2-simulação-do-conversor)
- [3. Sintonia de Ziegler-Nichols (Ultimate Gain)](#3-sintonia-de-ziegler-nichols-ultimate-gain)
  - [3.1 Ganho de oscilação](#31-ganho-de-oscilação)
  - [3.2 Cálculo dos Parâmetros PI](#32-cálculo-dos-parâmetros-pi)
- [4. Controle Digital (via Microcontrolador)](#4-controle-digital-via-microcontrolador)
  - [4.1 Discretização](#41-discretização)
  - [4.2 Implementação em Código](#42-implementação-em-código)
- [Observações Importantes](#observações-importantes)
- [Ferramentas Utilizadas](#ferramentas-utilizadas)
- [Anexo A (Discretização do Controlador PI, e Equação de Diferenças)](#anexo-a-discretização-do-controlador-pi-e-equação-de-diferenças)

---




### 1. Visão Geral

A metodologia apresentada a seguir tem como objetivo permitir o projeto de um controlador de forma simples e rápida, sem a necessidade de cálculos complexos, como a obtenção do modelo em pequenos sinais do conversor. A proposta é atender aos requisitos mínimos de controle com base apenas no conhecimento do dimensionamento dos elementos do conversor e na realização de simulações sobre seu comportamento. 



### 2. Simulação do Conversor

-Inicialmente, é necessário simular o conversor em um software. Neste caso, utilizaremos o **PSIM**, pois ele permite o uso de blocos de programação em C, o que será útil em etapas posteriores. Simularemos um **conversor Buck** com tensão de entrada de $50\ \text{V}$ e saída desejada de $25\ \text{V}$. De acordo com a fórmula do ganho estático desse conversor, o duty cycle ideal é $0{,}5$. Naturalmente, será projetado um controlador para garantir que a saída siga a referência, mesmo diante de possíveis perturbações no circuito.

<p align="center">
  <img width="687" height="298" alt="image" src="https://github.com/user-attachments/assets/3260dda9-539b-4f49-aaac-5fd6de66122c" />
</p>

### 3 Sintonia de Ziegler-Nichols _(Ultimate Gain)_

#### 3.1 Ganho de oscilação


- O primeiro passo consiste na implementação de um controle proporcional puro, ou seja um ganho na malha de realimentação.

<p align="center">
<img width="586" height="222" alt="image" src="https://github.com/user-attachments/assets/1c48935e-5827-4bef-80a2-85b558ec102e" />
</p>
  
- A constante de ganho proporcional (K) é aumentada até que o sistema entre em **estabilidade marginal**, ou seja, oscila com amplitude constante e sem crescimento, este estado está entre a estabilidade e a instabilidade.
- Para o caso do conversor Buck não é possível alcançar a instabilidade, pois a tensão de saída é no maximo igual a de entrada, assim o ponto de estabilidade marginal é o menor ganho que causa oscilações controladas.
- Isso fornece os parâmetros críticos:
  - **Ku**: ganho no ponto de oscilação marginal;
  - **Pu**: período da oscilação marginal (em segundos).
  
<p align="center">
<img width="668" height="388" alt="image" src="https://github.com/user-attachments/assets/3566d68d-5427-403c-91b4-6e77ee8b2c6e" />
</p>

<p align="center">
<img width="748" height="225" alt="image" src="https://github.com/user-attachments/assets/30124bb0-f522-42df-ad1d-74336e9af3a7" />
</p>

Aqui encontramos:
```
Ku = 0.37 
Pu = 6.05487e-04
```
#### 3.2 Cálculo dos Parâmetros PI

Em posse de Ku e Pu é possível cálcular valores de P/PI/PID a partir da tabela a seguir:

<p align="center">
<img width="406" height="200" alt="image" src="https://github.com/user-attachments/assets/b6f4e570-0224-481c-bd5d-85f784c78a89" />
</p>

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

- **Ganho (k)**: valor de Kp calculado.
- **Constante de Tempo (T)**: igual a Ti.
- É importante estabelecer os limites inferior e superior do seu bloco PI, de forma que se adeque ao _Duty Cycle_


<p align="center">
<img width="400" height="217" alt="image" src="https://github.com/user-attachments/assets/6cd5a2bf-1c47-411e-828a-c9f7f7017e56" />
</p>

<p align="center">
<img width="618" height="368" alt="image" src="https://github.com/user-attachments/assets/a2037865-b593-457e-b565-a52b842c0e4b" />
 </p>

<p align="center">
<img width="500" height="225" alt="image" src="https://github.com/user-attachments/assets/c2d191a2-133e-4aae-aacb-046b4861d302" />
 </p>

### 4. Controle Digital (via Microcontrolador)

#### 4.1 Discretização

Nesta seção, abordamos o processo de conversão do controlador do meio analógico para o digital, permitindo sua implementação em um microcontrolador.

No **MATLAB**, utilizamos a transformação bilinear para discretizar a função de transferência do controlador PI — obtida no domínio de Laplace a partir do bloco PI do PSIM — para o domínio Z. A discretização é feita considerando um período de amostragem de  $Ts = 10us$, valor adotado para o nosso controlador digital.

A partir da função de transferência discreta, extraímos os coeficientes necessários para a implementação da **equação de diferenças**, que representa o controlador em forma algébrica. Para o caso de um controlador PI, a função no domínio Z e sua correspondente equação de diferenças assumem a forma apresentada a seguir (para mais detalhes, consulte o anexo).

$$
G_c(z) = \frac{b_0 + b_1 z^{-1}}{1 + a_1 z^{-1}}
$$

$$
u[n] = -a_1 u[n-1] + b_0 e[n] + b_1 e[n-1]
$$

```matlab
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
```




#### 4.2 Implementação em Código
Por fim, implementamos a equação de diferenças no bloco Simplified C do PSIM, que simula o comportamento de um microcontrolador. Ainda dentro do código, geramos uma onda triangular de 5 kHz para compará-la com a saída do controlador PI, resultando no sinal PWM de controle.

Na simulação, o passo de cálculo adotado foi de 1 µs. Esse valor deve ser considerado tanto na geração da onda triangular quanto na lógica de controle, que foi projetada para operar com um período de amostragem de 10 µs.

A seguir, apresentamos o código utilizado:

```c
static double u_n, e_n;
static double u_n_1 = 0;
static double e_n_1 = 0;
static double tempo = 0;
const int V_ref = 25;

static double triangular = 0;
static double PI = 0;
static int a = 1;

double controle(double error) {
    tempo = 0;
    e_n = error;
    u_n = u_n_1 + 0.16814962 * e_n - 0.16485038 * e_n_1;
    u_n_1 = u_n;
    e_n_1 = e_n;
    return u_n;
}

double onda_triangular(void) {
    triangular = triangular + 2 * 0.01 * a;

    if (triangular >= 1) {
        a = -1;
    }

    if (triangular <= -1) {
        a = 1;
    }
}

onda_triangular();
tempo = tempo + delt;
if (tempo >= 0.000010) {
    PI = controle(V_ref - x1);
}

if (PI > triangular) {
    y1 = 1;
} else if (PI < triangular) {
    y1 = 0;
}
```

<p align="center">
<img width="945" height="426" alt="image" src="https://github.com/user-attachments/assets/d36686b3-0c11-4593-ba1a-3f2d726194b5" />
 </p>

### Observações Importantes

- É viável utilizar outros softwares para este projeto comoo LTspice, MATLAB, Proteus.
- A seleção de `Ts` é fundamental: geralmente é 10 a 20 vezes mais rápido que a dinâmica dominante do sistema.

---

### Ferramentas Utilizadas

- **PSIM** para simulações analógicas.
- **MATLAB** para análise de controle e discretização.
- **C** (ou similar) para implementação embarcada no microcontrolador.

---

Este projeto documenta uma implementação educativa de controle PI para conversores chaveados. Para o caso de dúvidas ygor.pereira@ee.ufcg.edu.br

### Anexo A (Discretização do Controlador PI, e Equação de Diferenças)

##### 1. Controlador PI no domínio contínuo

$$
G_c(s) = K_p + \frac{K_i}{s} = K_p \left(1 + \frac{1}{T_i s} \right)
$$

##### 2. Transformação Bilinear (Tustin)

$$
s \approx \frac{2}{T_s} \cdot \frac{1 - z^{-1}}{1 + z^{-1}} \quad \Rightarrow \quad
\frac{1}{s} \approx \frac{T_s}{2} \cdot \frac{1 + z^{-1}}{1 - z^{-1}}
$$

##### 3. Substituindo no controlador PI

$$
G_c(z) = K_p + K_i \cdot \frac{T_s}{2} \cdot \frac{1 + z^{-1}}{1 - z^{-1}}
$$

$$
G_c(z) = \frac{K_p(1 - z^{-1}) + \frac{K_i T_s}{2}(1 + z^{-1})}{1 - z^{-1}}
$$

$$
G_c(z) = \frac{(K_p + \frac{K_i T_s}{2}) + (-K_p + \frac{K_i T_s}{2}) z^{-1}}{1 - z^{-1}}
$$



$$
G_c(z) = \frac{b_0 + b_1 z^{-1}}{1 + a_1 z^{-1}}
$$

##### 4. Equação de Diferença

A equação de diferenças que implementa o PI discreto é:

$$
u[n] = -a_1 u[n-1] + b_0 e[n] + b_1 e[n-1]
$$

