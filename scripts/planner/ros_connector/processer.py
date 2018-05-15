class Processer:
    # gazebo coords and koef of sign blow
    def __init__(self, gazX, gazY, koef):
        self.mapX = gazX*koef
        self.mapY = gazY*koef
        self.koef = koef

    def to_gazebo(self, x, y):
        if x < self.mapX // 2 and y <self.mapY // 2:
            gaz_y = self.mapX // self.koef - x//self.koef *2
            gaz_x = self.mapY // self.koef - y//self.koef *2
        elif x > self.mapX // 2 and y <self.mapY // 2:
            gaz_y = (self.mapX - x) // self.koef *2 - (self.mapX // self.koef)
            gaz_x = self.mapY//self.koef - y//self.koef*2
        elif x > self.mapX // 2 and y > self.mapY // 2:
            gaz_y = (self.mapX - x) // self.koef*2 - (self.mapX // self.koef)
            gaz_x = (self.mapY - y) // self.koef*2 - (self.mapY // self.koef)
        else:
            gaz_y = self.mapX // self.koef - x // self.koef*2
            gaz_x = (self.mapY - y) // self.koef*2 - (self.mapY // self.koef)

        return gaz_x, gaz_y

    def to_signs(self, x, y):
        if x> 0 and y > 0:
            sig_y = self.mapY //2 - x*self.koef//2
            sig_x = self.mapX //2 - y*self.koef//2
        elif x >0 and y < 0:
            sig_y = self.mapY //2 - x*self.koef//2
            sig_x = self.mapX //2 + abs(y)*self.koef//2
        elif x<0 and y <0:
            sig_y = self.mapY // 2 + abs(x) * self.koef //2
            sig_x = self.mapX // 2 + abs(y) * self.koef //2
        else:
            sig_y = self.mapY // 2 + abs(x) * self.koef //2
            sig_x = self.mapX // 2 - y * self.koef // 2
        return sig_x, sig_y

if __name__ == "__main__":

    pr = Processer(10, 10, 20)


    #print(pr.to_gazebo(140, 140))
    print(pr.to_signs(-8, 8))