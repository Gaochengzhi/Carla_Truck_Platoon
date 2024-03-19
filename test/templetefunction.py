class BaseClassT:
    def __init__(self) -> None:
        here = 1

    def doinit(self):
        self.func1()
        self.func2()
    def func1(self):
        print("this is base 1")
    def func2(self):
        print("this is base 2")

bs = BaseClassT()
bs.doinit()

class sonClass(BaseClassT):
    def __init__(self) -> None:
        super().__init__()
    def func1(self):
        print("this is sbase 1")
    def func2(self):
        print("this is sbase 2")
    

bs = sonClass()
bs.doinit()