# Vector Fields

Os campos definidos a seguir foram criados com o intuito de melhorar o planejamento de trajetória para o ThunderVolt e são baseados na implementação da equipe [NeonFC](https://github.com/project-neon/NeonFC/wiki/Potential-Field). Existem 3 campos básicos: radial, linear e tangencial, porém seria interessante já fazer alguns campos úteis e mais complexos a partir deles, como o campo de evitar paredes e o de seguir a bola.

Uma observação importante, para evitar ter que manter referências a todos os campos dentro do seu programa e atualizar cada um por vez a cada frame, todos os parâmetros que descrevem os campos suportam objetos "chamáveis". Ou seja, dentro da função compute, esses objetos seram chamados, passando como parâmetro o `field_data`, de forma que todos os parâmetro podem ser atualizados automaticamente, sendo necessário apenas passar no construtor uma função com assinatura adequada em vez de um valor.

# vector_fields.VectorField

Classe mãe que implementa um campo vetorial genérico composto pela soma de outros campos.

Base: `object`

```python
class thundervolt.vector_fields.VectorField(field_data, **kwargs)
```

| Parâmetro | Tipo | Descrição |
| :--- |    :---   |   :--- |
| field_data | thundervolt.core.data.FieldData | Posição e velocidade de todas as entidade do jogo. |
| name | str | Nome do campo, por default será gerado automaticamente.  |


| Variável | Tipo | Descrição |
| :--- |    :---   |   :--- |
| output   | numpy.array | Último resultado fornecido pelo campo. |
| field_childrens | list | Conjunto de campos que compõem o campo genérico. |

## add(field)

| Parâmetro | Tipo | Descrição |
| :--- |    :---   |   :--- |
| field | thundervolt.vector_fields.VectorField | Campo que será adicionado. |

## compute(pose)

| Parâmetro | Tipo | Descrição |
| :--- |    :---   |   :--- |
| pose | thundervolt.core.data.Pose2D | Posição em que será calculado o campo. |


# vector_fields.RadialField

Classe que implementa um campo vetorial radial.

Base: `VectorField`

```python
class thundervolt.vector_fields.RadialField(field_data, **kwargs)
```

| Parâmetro | Tipo | Descrição |
| :--- |    :---   |   :--- |
| field_data | thundervolt.core.data.FieldData | Posição e velocidade de todas as entidade do jogo. |
| name | str | Nome do campo, por default será gerado automaticamente.  |
| target | tuple / list / numpy.array | Coordenadas xy do foco do campo radial. |
| repelling | bool | True se o campo for repulsivo. Default `False`. |
| max_radius | float | Raio até onde o campo tem efeito. Default `None`. |
| decay_radius | float | Raio a partir do qual o campo começa a decair linearmente. Default `None`. |
| field_limits | tuple / list | Define um retângulo onde o campo será válido. Default `None`. |
| multiplier | float | Número entre 0 e 1 que indica o quão forte aquele campo será. Ou sejá, é o valor máximo do campo. Default `1.0`. |

## compute(pose)

| Parâmetro | Tipo | Descrição |
| :--- |    :---   |   :--- |
| pose | thundervolt.core.data.Pose2D | Posição em que será calculado o campo. |


# vector_fields.LineField

Classe que implementa um campo vetorial paralelo a partir de uma linha.

Base: `VectorField`

```python
class thundervolt.vector_fields.LineField(field_data, **kwargs)
```

| Parâmetro | Tipo | Descrição |
| :--- |    :---   |   :--- |
| field_data | thundervolt.core.data.FieldData | Posição e velocidade de todas as entidade do jogo. |
| name | str | Nome do campo, por default será gerado automaticamente. |
| target | tuple / list / numpy.array | Coordenadas xy do centro da linha utilizada como base para o campo. |
| theta | float | Ângulo em radianos que define a direção da reta. Atenção que o sentido aqui pode ser relevante dependendo dos outro parâmetros. Ou seja, 0 e PI podem ter resultados diferentes. |
| size | float | Tamanho da linha que será desenhada a partir de `target` na direção `theta`. |
| only_forward | bool | Quando `True`, a linha será desenhada apenas na direção positiva de `theta`. Default `False`. |
| side | str | Define de qual lado da linha haverá campo. Pode ser `positive`, `negative` ou `both`. Default `both`. |
| repelling | bool | True se o campo for repulsivo. Default `False` |
| max_dist | float | Distância até onde o campo tem efeito. Default `None` |
| decay_dist | float | Distância a partir do qual o campo começa a decair linearmente. Default `None` |
| field_limits | tuple / list | Define um retângulo onde o campo será válido. Default `None` |
| multiplier | float | Número entre 0 e 1 que indica o quão forte aquele campo será. Ou sejá, é o valor máximo do campo. Default `1.0`. |

## compute(pose)

| Parâmetro | Tipo | Descrição |
| :--- |    :---   |   :--- |
| pose | thundervolt.core.data.Pose2D | Posição em que será calculado o campo. |


# vector_fields.TangentField

Classe que implementa um campo vetorial na forma de espiral hiperbólica.

Base: `VectorField`

```python
class thundervolt.vector_fields.TangentField(field_data, **kwargs)
```

| Parâmetro | Tipo | Descrição |
| :--- |    :---   |   :--- |
| field_data | thundervolt.core.data.FieldData | Posição e velocidade de todas as entidade do jogo. |
| name | str | Nome do campo, por default será gerado automaticamente.  |
| target | tuple / list / numpy.array | Coordenadas xy do foco do campo. |
| radius | float | Raio da espiral, corresponde ao raio que o robô orbitaria o foco. |
| clockwise | bool | Quando `True`, o sentido de rotação do campo é horário. Default `False` |
| damping | float | Constante de amortecimento da espiral. Quanto maior mais suave. Defualt `1/2500` |
| max_radius | float | Raio até onde o campo tem efeito. Default `None`. |
| decay_radius | float | Raio a partir do qual o campo começa a decair linearmente. Default `None`. |
| field_limits | tuple / list | Define um retângulo onde o campo será válido. Default `None`. |
| multiplier | float | Número entre 0 e 1 que indica o quão forte aquele campo será. Ou sejá, é o valor máximo do campo. Default `1.0`. |

## compute(pose)

| Parâmetro | Tipo | Descrição |
| :--- |    :---   |   :--- |
| pose | thundervolt.core.data.Pose2D | Posição em que será calculado o campo. |
