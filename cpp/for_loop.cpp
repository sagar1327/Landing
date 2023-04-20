#include <iostream>
int main()
{
    int sum = 0;
    
    for (int val = 51; val<100; ++val)
    {
        sum += val;
    }
    
    std::cout << "The total sum of the numbers between 50 to 100 is: "
              << sum << std::endl;
    
    return 0;
}
