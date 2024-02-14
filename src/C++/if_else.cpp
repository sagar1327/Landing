/*Given a positive integer n, do the following:
 -If 1 <= n <= 9, print the lowercase English word corresponding to the number (e.g., one for , two for etc.).
If n > 9, print Greater than 9.*/

#include <bits/stdc++.h>

using namespace std;

string ltrim(const string &); // Trims leading whitespace.
string rtrim(const string &); // Trims trialing whitespace.



int main()
{
    string n_temp;
    getline(cin, n_temp);

    int n = stoi(ltrim(rtrim(n_temp)));

    // Write your code here
    string names[9] = {"one","two","three","four","five","six","seven","eight","nine"};
    if (1 <= n and n <= 9){
        cout << names[n-1] << endl;
    }
    else{
        cout << "Greater than 9" << endl;
    }
}

string ltrim(const string &str) {
    string s(str);

    s.erase(
        s.begin(),
        find_if(s.begin(), s.end(), not1(ptr_fun<int, int>(isspace)))
    );

    return s;
}

string rtrim(const string &str) {
    string s(str);

    s.erase(
        find_if(s.rbegin(), s.rend(), not1(ptr_fun<int, int>(isspace))).base(),
        s.end()
    );

    return s;
}
