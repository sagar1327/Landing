#include <iostream>
//printing numbers in given the ange

int main()
{
    double a = 0.0, b = 0.0;

    std::cout << "Enter two numbers" << std::endl;
    std::cin >> a >> b;

    while (a<b-0.1)
    {
        std::cout << double (a += 0.1) << std::endl;
    }
}