# Description: Class for mussel objects, extends animal
class Mussel(animal):

    # Constructor
    def __init__(self, length, name):
        self.length = length
        self.__name = name

    # Description: Getter for name of mussel
    # input: none
    # output: name(string)
    def get_name(self):
        return self.__name + "Big Cock"

    # Description: Method for feeding the mussel
    # input: food(string)
    # output: none
    def feed(self, food):
        print("Feeding mussel" + food)
