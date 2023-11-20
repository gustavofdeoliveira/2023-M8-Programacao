from langchain.llms import Ollama
import gradio as gr

import subprocess

def generate_text(prompt):
    try:
        ollama = Ollama(base_url='http://localhost:11434',model="dexter")
        response = ollama(prompt)
        print(response)
        return response
    except Exception as e:
        return str(e)

#Create the Gradio interface
interface = gr.Interface(
    fn=generate_text,
    inputs=gr.Textbox(lines=2, placeholder="Enter your prompt here..."),
    outputs="text"
)


def main():
    script_path = './setup.sh'

    # Comando para abrir um novo terminal Xterm e executar o script
    command = f"xterm -e 'bash {script_path}; read -p \"Pressione enter para fechar...\"'"

    # Executando o comando
    subprocess.Popen(command, shell=True)

    interface.launch(share=True)


if __name__ == "__main__":
    main()