# Controle de Conversores DC-DC Buck com Controlador PI

Este repositório documenta o processo completo de projeto, simulação e implementação de um controlador PI para um conversor DC-DC do tipo buck, utilizando o método de sintonia de Ziegler-Nichols.

---

## Etapas do Projeto

### 1. Projeto Analógico

#### 1.1. Estabilidade Marginal

- Inicialmente, foi implementado um controle proporcional puro.
- A constante de ganho proporcional (K) foi aumentada até que o sistema entrasse em **estabilidade marginal**, ou seja, oscila com amplitude constante e sem crescimento.
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

