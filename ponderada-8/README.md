## Ponderada 8: STT e TTS

### Descrição Geral

Este projeto Python realiza duas funções principais:

1. **Transcrição de Áudio**: Converte arquivos de áudio em texto.
2. **Interação com LLM**: Traduz o texto transcritos para um idioma desejado.
3. **Tradução de Texto**: Traduz o texto transcritos para um idioma desejado.

### Dependências

- `os`
- `pathlib`
- `playsound`
- `openai`

### Instalação e Configuração

#### 1. Clonando o Repositório

Inicialmente, clone o repositório para a sua máquina local:

```bash
git clone https://github.com/gustavofdeoliveira/2023-M8-Programacao
cd ponderada-8/src
```

#### 2. Configurando o Ambiente Virtual

Recomenda-se criar um ambiente virtual para gerenciar as dependências. Para isso, execute:

```bash
python3 -m venv .
source venv/bin/activate  
```
No Windows use `source Scripts\activate`

#### 3. Instalando as Dependências

Instale as dependências necessárias usando `pip`:

```bash
pip install -r requirements.txt
```

> **Nota**: `requirements.txt` deve ser fornecido com todas as dependências necessárias.

#### 4. Configuração da API do OpenAI

Adicione sua chave de API do OpenAI:

- Crie um arquivo `.env` na raiz do projeto.
- Adicione a seguinte linha ao arquivo: `OPENAI_API_KEY="SuaChaveDeAPIAqui"`.


### Executando o Projeto
Agora, você pode executar o projeto usando o seguinte comando:

```bash
python main.py
```

### Exemplo de Uso

[Clique aqui para acessar o vídeo](https://drive.google.com/file/d/1L70MBRjivXX6CFmXVsSBdbO0Ak5CDdVn/view?usp=drive_link)



