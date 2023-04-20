#include <iostream>
/*Count repeating numbers.*/

int main()
{
    int num = 0; int count = 0; int vector[10] /*the list of numbers*/; int dump[10] /*a dump list for for avoiding prinitng same number*/;
    
    for(int i = 0; i<=9; i++)
    {
        std::cin >> vector[i];
        dump[i] = 0; //Initially the dump list will have all element as 0.
    }

    //Main loop
    for (int i=0;i<=9;i++)
    {
        //Loop for counting the repeatition
        for (int j=0;j<=9;j++)
        {
            
            if (vector[i]==vector[j])
            {
                count+=1;
            }
        }
    
        //Statement to fill out the spaces of that repeated numeber to avoid printing same number.
        if (dump[i]==0)
        {
            for (int k=0;k<=9;k++)
            {
                if (vector[k] == vector[i])
                {
                    dump[k] = 1; //1 means that space is occupied. So, avoid printing that number again.
                }  
            }
            std::cout << "\n" <<vector[i] << " occured " << count << " times." << std::endl;
        }
        count=0; //re-allocating the value 0 to count
        
    }        
            
    return 0;
}