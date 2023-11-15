import re
import rclpy
from rclpy.node import Node

# Dicionário mapeando expressões regulares a identificadores de intenção
intent_dict = {
    # Reconhece saudações como "Bom dia", "Boa tarde", "Boa noite"
    r"\b(?:(?:[Bb]o[am])\s(tarde|dia|noite))": "bom",
    # Reconhece perguntas sobre o estado de ânimo
    r"\b(?:[Tt]udo)?\s?(?:(?:[bB]em)|(?:[bB]ão)|(?:[fF]irme)|(?:em\sriba))\?": "bem",
    # Reconhece comandos para ir à secretaria
    r"\b(?:(?:[Vv]á\spara\s)|(?:[Mm]e\sleve\saté\s))a\ssecretaria\b": "secretaria",
    # Reconhece comandos para ir ao laboratório
    r"\b(?:(?:[Vv]á\spara\s)|(?:[Mm]e\sleve\saté\s))o\slaboratório\b": "laboratorio",
    # Reconhece comandos para ir à biblioteca
    r"\b(?:(?:[Vv]á\spara\s)|(?:[Mm]e\sleve\saté\s))a\sbiblioteca\b": "biblioteca",
    # Reconhece saudações simples como "Oi" ou "Olá"
    r"\b(?:[Oo]i|[Oo]lá)\b": "saudacao"
}

# Classe base para ações
class Action:
    # Executa a ação baseada no contexto do comando
    def execute(self, context):
        found_action = False
        for key, value in intent_dict.items():
            pattern = re.compile(key)
            groups = pattern.findall(context)
            if groups:
                # Se uma correspondência é encontrada, executa a ação associada
                action = ActionFactory.get_action(value)
                print(f"{action.execute(groups[0])} \n", end=" ")
                found_action = True
        # Se nenhuma ação corresponder, exibe uma mensagem de erro
        if not found_action:
            print("Desculpe, não consegui entender! \n", end=" ")

# Classes para diferentes tipos de ações
class HelloBack(Action):
    # Executa a ação de saudação
    def execute(self, hello):
        return f"{hello}, tudo bem?"

class DayBack(Action):
    # Executa a ação de resposta a saudações específicas do tempo do dia
    def execute(self, time_of_day):
        if time_of_day == "dia":
            return f"Bom {time_of_day}!"
        else:
            return f"Boa {time_of_day}!"

class HowBack(Action):
    # Executa a ação de resposta a perguntas sobre o estado de ânimo
    def execute(self, _):
        return "Comigo está tudo bem, e com você?"
    
class SecretaryBack(Action):
    # Executa a ação de direcionamento para a secretaria
    def execute(self, _):
        return "Indo para a secretaria, passando pelos pontos - X: 1.5 - Y:1.7 - Z: 0.0"
    
class LaboratoryBack(Action):
    # Executa a ação de direcionamento para o laboratório
    def execute(self, _):
        return "Indo para o laboratorio, passando pelos pontos - X: 0.3 - Y:0.7 - Z: 0.0"
    
class LibraryBack(Action):
    # Executa a ação de direcionamento para a biblioteca
    def execute(self, _):
        return "Indo para a biblioteca, passando pelos pontos - X: -1.1 - Y:-1.5 - Z: 0.0"

# Fábrica para criar a ação apropriada com base no tipo de ação
class ActionFactory:
    @staticmethod
    def get_action(action_type):
        # Mapeia o tipo de ação para a classe de ação correspondente
        if action_type == "saudacao":
            return HelloBack()
        elif action_type == "bom":
            return DayBack()
        elif action_type == "bem":
            return HowBack()
        elif action_type == "secretaria":
            return SecretaryBack()
        elif action_type == "laboratorio":
            return LaboratoryBack()
        elif action_type == "biblioteca":
            return LibraryBack()
        else:
            raise "Desculpe não consegui entender!"

# Classe principal para o nó do chatbot
class ChatBotNode(Node):
    def __init__(self):
        super().__init__('chat_bot')
        self.action = Action()
        while True:  # Loop infinito para manter a solicitação de comandos
            print("Caso queira encerrar o programa digite 'sair' \n", end=" ")
            self.command = input("Digite o seu comando: ")
            if self.command.lower() == 'sair':  # Verifica a condição de saída
                exit()
            self.action.execute(self.command)

# Função principal para executar o nó do chatbot
def main(args=None):
    rclpy.init(args=args)
    chatbot = ChatBotNode()
    rclpy.spin(chatbot)
    chatbot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()