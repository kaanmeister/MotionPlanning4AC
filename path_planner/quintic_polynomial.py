import numpy as np
import matplotlib.pyplot as plt 
class QuinticPolynomial:
    def __init__(self, p0, p1, p2, p3, p4, p5, s_start, s_end):
        self.p0, self.p1, self.p2, self.p3, self.p4, self.p5 = p0, p1, p2, p3, p4, p5
        
        self.s_start = s_start
        self.s_end = s_end

    def calc_points(self, s):
        s_relative = s - self.s_start


        result = (self.p5 * s_relative**5 + self.p4 * s_relative**4 + self.p3 * s_relative**3 + self.p2 * s_relative**2 + self.p1 * s_relative + self.p0)
    
        return result


    def calc_first_derivative(self, s):
        s_relative = s - self.s_start

        result = (5 * self.p5 * s_relative**4 + 4 * self.p4 * s_relative**3 + 3 * self.p3 * s_relative**2 + 2 * self.p2 * s_relative + self.p1)

        return result


    def calc_sec_derivative(self, s):
        s_relative = s - self.s_start

        result = (20 * self.p5 * s_relative**3 + 12 * self.p4 * s_relative**2 + 6 * self.p3 * s_relative + 2 * self.p2)

        return result

        


    def calc_third_derivative(self,s):
        s_relative = s - self.s_start

        result = (60 * self.p5 * s_relative**2 + 24 * self.p4 * s_relative + 6 * self.p3)

        return result

if __name__ == '__main__':

    p0, p1, p2, p3, p4, p5, = 0, 0, 0, 0.1, -0.01, 0.002

    poly_segment = QuinticPolynomial(p0, p1, p2, p3 , p4, p5, s_start=0, s_end=5)

    s_values = np.linspace(poly_segment.s_start, poly_segment.s_end, 100)

    l_values = [poly_segment.calc_points(s) for s in s_values]

    plt.figure(figsize=(10, 6))
    plt.plot(s_values, l_values, label="Quintic Polynomial Segment", color='b', linewidth=2)
    plt.title("Test of QuinticPolynomial Class")
    plt.xlabel("s (distance along reference line)")
    plt.ylabel("l (lateral offset)")
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.show()