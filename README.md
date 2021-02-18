# 🔥 FIRA ThunderVolt ⚽

![pytest CI](https://github.com/ThundeRatz/fira_thundervolt/workflows/pytest%20CI/badge.svg)

Implementação especial do ThunderVolt para o simulador FIRASim

## Ambiente virtual de Python

Para rodar o projeto em um ambiente virtual com o módulo [venv](https://docs.python.org/pt-br/3/library/venv.html#module-venv), primeiramente instale-o da seguinte forma:

```bash
sudo apt install python3-venv
```

Então para criar o abiente virtual faça:

```bash
python3 -m venv venv
```

Então para ativar o ambiente virtual pelo `bash`, faça:

```bash
source ./venv/bin/activate
```

Já pelo fish, faça:

```bash
source ./venv/bin/activate.fish
```

Por fim, caso o arquivo `requirements.txt` não exista ou caso seja necessário atualizá-lo, com todas as depências necessárias instaladas manualmente no ambiente virtual, gere o arquivo fazendo:

```bash
pip3 freeze -l > requirements.txt
```

Caso o arquivo  já exista, para instalar as dependências, rode o seguinte comando:

```bash
pip3 install -r requirements.txt
```

Para desativar o ambiente virtual, rode no terminal:

```bash
deactivate
```

## Rodando o pacote

Para rodar o pacote, estando na raiz do repositório, faça o seguinte:

```bash
python3 -m thundervolt
```

## Rodando os testes

### Testes de integração

Para rodar os testes de integração, rode o seguinte comando:

```bash
python3 ./integration_tests/nome_do_teste.py
```

### Testes unitários

Para rodar os testes unitários, rode o seguinte comando:

```bash
python3 -m pytest
```
