import numpy as np


class Perceptron:
    def __init__(self, learning_rate=0.1, n_iterations=100, threshold=0.5):
        self.learning_rate = learning_rate
        self.n_iterations = n_iterations
        self.threshold = threshold
        self.weights = np.zeros(2)
        self.bias = 0

    def activation_function(self, x):
        return 1 if x >= self.threshold else 0

    def predict(self, inputs):
        linear_output = np.dot(inputs, self.weights) + self.bias
        y_predicted = self.activation_function(linear_output)
        return y_predicted

    def train(self, X, y):
        for _ in range(self.n_iterations):
            for x, y_true in zip(X, y):
                y_pred = self.predict(x)
                error = y_true - y_pred
                self.weights += error * self.learning_rate * x
                self.bias += error * self.learning_rate


class PerceptronAnd():
    def __init__(self):
        self.gate_and_X = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
        self.gate_and_y = np.array([0, 1, 1, 0])

    def train(self):
        self.perceptron = Perceptron()
        self.perceptron.train(self.gate_and_X, self.gate_and_y)

    def run(self):
        self.train()
        print("Porta AND")
        print(f"in (0, 0), out: {self.perceptron.predict([0, 0])}")
        print(f"in (0, 1), out: {self.perceptron.predict([0, 1])}")
        print(f"in (1, 0), out: {self.perceptron.predict([1, 0])}")
        print(f"in (1, 1), out: {self.perceptron.predict([1, 1])}")


class PerceptronOr():
    def __init__(self):
        self.gate_or_X = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
        self.gate_or_y = np.array([0, 1, 1, 1])

    def train(self):
        self.perceptron = Perceptron()
        self.perceptron.train(self.gate_or_X, self.gate_or_y)

    def run(self):
        self.train()
        print("Porta OR")
        print(f"in (0, 0), out: {self.perceptron.predict([0, 0])}")
        print(f"in (0, 1), out: {self.perceptron.predict([0, 1])}")
        print(f"in (1, 0), out: {self.perceptron.predict([1, 0])}")
        print(f"in (1, 1), out: {self.perceptron.predict([1, 1])}")

class PerceptronNand():
    def __init__(self):
        self.gate_nand_X = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
        self.gate_nand_y = np.array([1, 1, 1, 0])

    def train(self):
        self.perceptron = Perceptron()
        self.perceptron.train(self.gate_nand_X, self.gate_nand_y)

    def run(self):
        self.train()
        print("Porta NAND")
        print(f"in (0, 0), out: {self.perceptron.predict([0, 0])}")
        print(f"in (0, 1), out: {self.perceptron.predict([0, 1])}")
        print(f"in (1, 0), out: {self.perceptron.predict([1, 0])}")
        print(f"in (1, 1), out: {self.perceptron.predict([1, 1])}")

class PerceptronXor():
    def __init__(self):
        self.gate_xor_X = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
        self.gate_xor_y = np.array([0, 1, 1, 0])
        self.message = "A porta XOR retorna 1 quando o número de entradas verdadeiras é ímpar, como quando uma entrada é 1 e a outra é 0, e 0 em outros casos. Não é possível separar estes casos com uma linha reta em um gráfico, indicando a não-linearidade da XOR. Para simular XOR em redes neurais, é necessário usar uma rede multicamadas com pelo menos uma camada oculta contendo dois neurônios, permitindo aprender fronteiras de decisão não-lineares."

    def train(self):
        self.perceptron = Perceptron()
        self.perceptron.train(self.gate_xor_X, self.gate_xor_y)

    def run(self):
        self.train()
        print("Porta XOR")
        print(f"in (0, 0), out: {self.perceptron.predict([0, 0])}")
        print(f"in (0, 1), out: {self.perceptron.predict([0, 1])}")
        print(f"in (1, 0), out: {self.perceptron.predict([1, 0])}")
        print(f"in (1, 1), out: {self.perceptron.predict([1, 1])}")
        print(self.message)



def main():
    perceptron_and = PerceptronAnd()
    perceptron_and.run()
    perceptron_or = PerceptronOr()
    perceptron_or.run()
    perceptron_nand = PerceptronNand()
    perceptron_nand.run()
    perceptron_xor = PerceptronXor()
    perceptron_xor.run()


# Exemplo de uso
if __name__ == "__main__":
    main()
