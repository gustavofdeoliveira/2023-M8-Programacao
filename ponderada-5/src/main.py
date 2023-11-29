import gradio as gr
from langchain.llms import Ollama
from langchain.prompts import ChatPromptTemplate
from langchain.schema.runnable import RunnableLambda, RunnablePassthrough
from langchain.document_loaders import TextLoader
from langchain.embeddings.sentence_transformer import SentenceTransformerEmbeddings
from langchain.text_splitter import CharacterTextSplitter
from langchain.vectorstores import Chroma


loader = TextLoader("./data/arquivo_entrada.txt")

documents = loader.load()

text_splitter = CharacterTextSplitter(chunk_size=1000, chunk_overlap=0)
docs = text_splitter.split_documents(documents)

# create the open-source embedding function
embedding_function = SentenceTransformerEmbeddings(model_name="all-MiniLM-L6-v2")

# load it into Chroma
vectorstore = Chroma.from_documents(docs, embedding_function)

retriever = vectorstore.as_retriever()

template = """Answer the question based only on the following context:
{context}

Question: {question}
"""
prompt = ChatPromptTemplate.from_template(template)

model = Ollama(model="mistral:7b-instruct-q3_K_S")

chain = (
    {"context": retriever, "question": RunnablePassthrough()}
    | prompt
    | model
)

def response(message, history):
    response = ""
    for response_message in chain.stream(message):
        response += response_message
        yield response

def main():
    gradio = gr.ChatInterface(response).queue()
    gradio.launch(share=True)


if __name__ == "__main__":
    main()