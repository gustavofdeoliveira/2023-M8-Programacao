import gradio as gr
from langchain.llms import Ollama
from langchain.prompts import ChatPromptTemplate
from langchain.schema.runnable import RunnableLambda, RunnablePassthrough
from langchain.document_loaders import TextLoader
from langchain.embeddings.sentence_transformer import SentenceTransformerEmbeddings
from langchain.text_splitter import CharacterTextSplitter
from langchain.vectorstores import Chroma

# Carrega o documento a partir de um arquivo de texto.
loader = TextLoader("./data/arquivo_entrada.txt")

# Carrega os documentos para o processamento.
documents = loader.load()

# Define um divisor de texto baseado em caracteres.
text_splitter = CharacterTextSplitter(chunk_size=1000, chunk_overlap=0)
docs = text_splitter.split_documents(documents)

# Cria uma função de embedding baseada em SentenceTransformer.
embedding_function = SentenceTransformerEmbeddings(model_name="all-MiniLM-L6-v2")

# Carrega os documentos em um vectorstore do Chroma.
vectorstore = Chroma.from_documents(docs, embedding_function)

# Cria um recuperador de documentos a partir do vectorstore.
retriever = vectorstore.as_retriever()

# Define um template para o prompt de chat.
template = """Answer the question based only on the following context:
{context}

Question: {question}
"""
prompt = ChatPromptTemplate.from_template(template)

# Inicializa o modelo Ollama.
model = Ollama(model="mistral:7b-instruct-q3_K_S")

# Define uma cadeia de processamento para as mensagens.
chain = (
    {"context": retriever, "question": RunnablePassthrough()}
    | prompt
    | model
)

# Define a função de resposta para o chat.
def response(message, history):
    response = ""
    for response_message in chain.stream(message):
        response += response_message
        yield response

# Define a função principal para executar a interface do Gradio.
def main():
    gradio = gr.ChatInterface(response).queue()
    gradio.launch(share=True)

# Executa a função principal se o script for o principal.
if __name__ == "__main__":
    main()
