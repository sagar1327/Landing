#include <iostream>
int main()
{
    //Target value and guess value
    int target = 23, guess = 0;
    std::cout <<"Guess the target value." << std::endl;
    /*While loop until the guess 
    value is not equal to target value*/
    while (true)
    {
        std::cin >> guess;
        if (guess == target)
        {
            std::cout <<"Congrats!You guessed it.\n\n";
            return 0;
        }
        else
        {
            std::cout <<"Opps!.Try again\n";
        }

    }

}