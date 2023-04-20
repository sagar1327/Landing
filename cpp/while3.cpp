#include <iostream>
/*Adding numbers until the user
stop inputting numbers*/

int main()
{
    int sum = 0; int num = 0;
    while(std::cin >> num)
    {
        sum += num;
    }

    std::cout << "Total sum is: " << sum << std::endl;
    return 0;
}