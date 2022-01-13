from abc import ABC, abstractmethod


class GripperListenerI(ABC):
    """
    Интерфейс Наблюдателя объявляет метод обработки, который использует
    интерфейс взаимодействия с ардуиной(динамикселем) для отправки полученных данных на обработку.
    """

    @abstractmethod
    def process_data(self, package: bytes, type_code: int, left_val: float, right_val: float) -> None:
        pass
