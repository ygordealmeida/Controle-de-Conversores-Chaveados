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

- A constante de ganho proporcional (K) é aumentada até que o sistema entrasse em **estabilidade marginal**, ou seja, oscila com amplitude constante e sem crescimento, este estado está entre a estabilidade e instabilidade.
- Para o caso do conversor Buck não é possível alcançar a instabilidade, pois a tensão de saída é no maximo igual a de entrada, assim o ponto de estabilidade marginal é o menor ganho que causa oscilações controladas.
- Isso fornece os parâmetros críticos:
  - **Ku**: ganho no ponto de oscilação marginal.
  - **Pu**: período da oscilação marginal (em segundos)

<img width="1336" height="776" alt="image" src="https://github.com/user-attachments/assets/3566d68d-5427-403c-91b4-6e77ee8b2c6e" />

<img width="1895" height="850" alt="image" src="https://github.com/user-attachments/assets/30124bb0-f522-42df-ad1d-74336e9af3a7" />


#### 1.1. Estabilidade Marginal

- Inicialmente, foi implementado um controle proporcional puro.
- A constante de ganho proporcional (K) foi aumentada até que o sistema entrasse em **estabilidade marginal**, ou seja, oscila com amplitude constante e sem crescimento.
- A estabilidade marginal fica entre a estabilidade e a instabilidade, para o caso do conversor Buck não é possível alcançar a instabilidade, pois a tensão de saída é no maximo igual a de entrada, assim o ponto de estabilidade marginal é o menor ganho que causa oscilações controladas.
- Isso fornece os parâmetros críticos:
  - **Ku**: ganho no ponto de oscilação marginal.
  - **Pu**: período da oscilação marginal (em segundos).



#### 1.2. Cálculo dos Parâmetros PI

Utilizando a forma padrão do método de Ziegler-Nichols para controladores PI:

```
Kp = 0.45 * Ku
Ti = Pu / 1.2
```

No PSIM, o bloco PI usa a forma:

```
G(s) = k * (1 + sT) / (sT)
```

Logo, configuramos:

- **Gain (k)**: valor de Kp calculado.
- **Time Constant (T)**: igual a Ti.

### 2. Anti-Windup

O Anti-windup é um mecanismo que impede que o integrador continue acumulando erro quando a saída do controlador está saturada. Isso é essencial para evitar instabilidades e recuperação lenta.

---

## Controle Digital (via Microcontrolador)

### 1. Estrutura do PI Digital

Usa-se a equação em diferenças:

```c
u[k] = u[k-1] + Kp*(e[k] - e[k-1]) + Ki*e[k];
```

Onde:

- `Kp` é igual ao do controle analógico.
- `Ki = Kp * Ts / Ti`, onde `Ts` é o tempo de amostragem do sistema.

### 2. Alternativa com Discretização em MATLAB

Em vez da equação manual, é possível discretizar a função de transferência PI analógica usando o MATLAB:

```matlab
s = tf('s');
Gpi = Kp * (1 + 1/(Ti*s));
Ts = 20e-6; % Exemplo
Gpi_d = c2d(Gpi, Ts, 'tustin');
```

- A função `c2d` retorna a função de transferência em z.
- Pode ser convertida em equação em diferenças via `tfdata` ou `filter`.

---

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

## Sugestões de Expansão

- Implementação com controle adaptativo.
- Comparativo entre PI e controle por média corrente.
- Versão com compensador em malha externa e interna (controle aninhado).

---

## Licença

MIT License

---

Este projeto documenta uma implementação educativa de controle PI para conversores chaveados. Se for útil para você, dê um star no repositório!

