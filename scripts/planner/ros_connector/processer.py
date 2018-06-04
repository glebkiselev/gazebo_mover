class Processer:
    # gazebo coords and koef of sign blow
    def __init__(self, gazX, gazY, koef):
        self.mapX = gazX*koef #200
        self.mapY = gazY*koef #200
        self.koef = koef #20

    def to_gazebo(self, x, y): #
        if x < self.mapX // 2 and y <self.mapY // 2:
            gaz_y = (self.mapY // 2 - y) / self.koef * 2
            gaz_x = (self.mapX // 2 - x) / self.koef * 2 * (-1)    # because in 2 quadro
        elif x > self.mapX // 2 and y <self.mapY // 2: # 1 quadro
            gaz_y = (self.mapY // 2 - y) / self.koef * 2
            gaz_x = (x - self.mapX // 2) / self.koef * 2
        elif x > self.mapX // 2 and y > self.mapY // 2: # 4 quadro
            gaz_y = (y - self.mapY // 2) / self.koef * 2 * (-1)
            gaz_x = (x - self.mapX // 2) / self.koef * 2
        else: # 3 quadro
            gaz_y = (y - self.mapY // 2) / self.koef * 2 * (-1)
            gaz_x = (self.mapX // 2 - x) / self.koef * 2 * (-1)
            if gaz_x == 0.0:
                gaz_x *= -1
            if gaz_y == 0.0:
                gaz_y *= -1

        return gaz_x, gaz_y

    def to_signs(self, x, y): # 2.5 2
        if x > 0.0 and y > 0.0: # 1 quadro
            sig_y = self.mapY //2 - y*self.koef//2
            sig_x = self.mapX //2 + x*self.koef//2
        elif x > 0.0 and y < 0.0: # 4 quadro
            sig_y = self.mapY //2 + abs(y)*self.koef//2
            sig_x = self.mapX //2 + x*self.koef//2
        elif x<0.0 and y <0.0: # 3 quadro
            sig_y = self.mapY //2 + abs(y)*self.koef//2
            sig_x = self.mapX //2 - abs(x)*self.koef//2
        else: # 2 quadro
            sig_y = self.mapY //2 - y*self.koef//2
            sig_x = self.mapX //2 - abs(x)*self.koef//2
        return sig_x, sig_y

if __name__ == "__main__":

    pr = Processer(10, 10, 20)


    #print(pr.to_gazebo(140, 140))
    print(pr.to_gazebo(160, 33))