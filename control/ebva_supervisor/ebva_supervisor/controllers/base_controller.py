from abc import ABC, abstractmethod

class BaseController(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def control(self, state_data) -> tuple:
        """
        Recebe os dados de estado e retorna (steering, throttle).
        Deve ser implementado obrigatoriamente pelas classes filhas.
        """
        pass