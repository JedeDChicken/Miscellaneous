// Setup- https://youtu.be/DMWD7wfhgNY

// Fixes- 
// ctrl + shift + P (CMD Palette)
// C/C++ Edit Configurations (UI)- fixed red squiggles in #include <iostream>
    // Compiler Path- C:/msys64/ucrt64/bin/g++.exe
    // IntelliSense Mode- windows-gcc-x64

// Preferences: Open User Settings (JSON)
    /* changed "bash (MSYS2)": {
            "path": "C:\\msys64\\usr\\bin\\bash.exe",
            "args": [
                "--login",
                "-i"
            ],
            "env": {
                "CHERE_INVOKING": "1"
            }
        }
    */
    /* to "MSYS2 UCRT64": {
            "path": "C:\\msys64\\usr\\bin\\bash.exe",
            "args": [
                "--login",
                "-i"
            ],
            "env": {
                "MSYSTEM": "UCRT64",
                "CHERE_INVOKING": "1"
            }
        }
    */
    // also changed "terminal.integrated.defaultProfile.windows": "PowerShell"
    // to "terminal.integrated.defaultProfile.windows": "MSYS2 UCRT64"

// ctrl + , > Code Runner Executor Map > Edit in settings.json
    // changed "cpp": "cd $dir && g++ $fileName -o $fileNameWithoutExt && $dir$fileNameWithoutExt"
    // to "cpp": "g++ \"$fileName\" -o \"$fileNameWithoutExt\" && ./$fileNameWithoutExt"

// Reinstallations in MSYS2 MINGW UCRT x64- fixed compiler
    // pacman -Syu
    // pacman -S mingw-w64-ucrt-x86_64-toolchain
    // pacman -S --needed base-devel mingw-w64-ucrt-x86_64-toolchain (removed --needed to force?)

// Checks- 
    // g++ --version, gcc --version, gdb --version
    // echo $MSYSTEM (must output UCRT64), which g++ (must output /ucrt64/bin/g++)
    // echo $? (must output 0)
    // g++ "<file_name>.cpp" -o "<file_name>.exe", ./<file_name>.exe (or just <file_name> instead of <file_name>.exe?)

// https://youtu.be/q1ZmFc-sqNc
    // C++ Principles- no language beneath C++ except Assembly, 
    // Only pay for what you use, high-lvl abstractions come at near 0 cost

    // C:structs (memory organized in particular order)
    // C++:objects ()

    // "C makes it easy to shoot yourself in the foot; 
    // C++ makes it harder, but when youu do it blows your whole leg off"
    // - by the language inventor
    // More complicated syntax that's more difficult to deciper... err msgs... template is prone to...
    // Avoid (syntactic) sugars... unnecessary features...

// https://youtu.be/kr8Hb9idWqo
// 1. Learn code- pick any 1 you like online...
// 2. After basics- care pitfalls, learn how to actually code (do code, challenging projects) not just theories
// 3. KISS (no need for weird C++ features, e.g. some smart pointers, templates [just use containers already implementded by C++], )
// 4. Avoid OOP? as much as possible?, Refactor..., 

// https://youtu.be/MNeX4EGtR5Y
// Static, compiled, C (low-level memory & h/w control) + “OOP” ()
// For systems w/ constrained memory demands- e.g. (since fast, mid-lvl lang) game engines, vid editors, embedded systems, db’s, compilers & VM’s
// Python’s written in C

/*
#include <iostream>  // Handles i/o
#include <string>  // To use string type (instead of char & arrays...)
using namespace std;  // std (Standard Char Output), namespace ()

// OOP
class Human {
    float dna;  // Private (by default) members (attributes/methods), can have default vals... can overwrite...

    public:  // To make public, Public Specifier
        string face;  // Use Getters&Setters to access publicly...
        void walk() {
            cout << "Walking...";
        }
        void walk(int speed) {  // Overloading (like form of Polymorphism, in C++)
            string speed_str = to_string(speed);
            cout << "Walking at " + speed_str + " speed..." << endl;
        }

        void eat();
};

void Human::eat() {  // Define methods outside class
    cout << "Eating... ";
};

// Constructors and Destructors- to run code when obj's created/destroyed
// Friend

// Inheritance
class Superhero : public Human {
    string hero = "Clint";

    public:
        string weapon = "Cape";
        void fly();
};

int main () {  // Main Function, Start
    // char hello[] = "Look at my string!";  // String variable (array of chars)
    string hello = "Look at my string!";

    cout << "Hello World!" << endl;  // << (Bitwise Shift Left)
    cout << hello << endl;

    Human Ally;  // Instantiate obj
    Ally.eat();
    Ally.walk(5);

    Superhero Katana;
    cout << Katana.weapon << endl;

    // Memory Management- Pointers & References, Deallocate Memory, Smart/Unique Pointers (safer, only 1 obj allocated to 1 memory)
    // malloc

    return 0;
}
*/

// Crash Course- https://youtu.be/RSDzvlXmQi4
// Functions- <return dtype> <funcName>(<params>) {}
// std::cout << "Hello, World!";
// << (insertion operator, left shift)- to format data accdg to dtype then send to cout obj
// Operator Overloading- give diff meanings to operators (like Polymorphism...)
// ::- Scope Resolution Operator...
// Namespace- to group related code together to avoid naming conflicts
// dtypes- int, double (float?), char (1char), string, bool
// Keywords- 
// cin >> <variable>;
// >> (extraction operator)

/*
#include <iostream>
Funcs & Classes- outside main() func
int add (int a, int b) {
    return a + b;
}

int main () {
    int a = 0;
    while (a<5) {
        std::cout << a << std::endl;
        a++;
    }

    do {
        std::cout << a << std::endl;
        a++;
    } while (a<5);  // Code is executed at least once

    for (int i=0; i<5; i++) {
        std::cout << i << std::endl;
    }

    int twitter = 10, threads = 5;
    if (twitter > threads) {
        std::cout << "Twitter Won!" << std::endl;
    }
    else if (twitter == threads) {
        std::cout << "Both won?" << std::endl;
    }
    else {
        std::cout << "Threads Won!" << std::endl;
    }

    int no_of_likes;
    std::cin >> no_of_likes;
    switch (no_of_likes) {
        case 10:
            std::cout << "Good start" << std::endl;
            break;
        case 100:
            std::cout << "Target achieved" << std::endl;
            break;
        default:
            std::cout << "Not enough info" << std::endl;
            break;
    }

    Function call
    std::cout << add(6, 7) << std::endl;

    return 0;
}
*/

// Full Course- https://youtu.be/-TkoO8Z07hI
#include <iostream>
#include <vector>  // For 'dynamic' arrays, for typedef e.g.
#include <cmath>  // For other operations
#include <ctime>  // Also for random #s
#include <iomanip>  // For decimal places?, std::setprecision(2) << std::fixed (2 decimal places...)
#include <algorithm>
// #include <string>
// using namespace std;  // Care since std has many entities...

// Namespace- to prevent conflicts in large projects, allows for identically named entities as long as namespaces are different
namespace first {
    int x = 1;
}
namespace second {
    int x = 2;
}

// typedef std::vector<std::pair<std::string, int>> pairlist_t;  // <>_t (convention for typedef), for paired list
// typedef std::string str_t;
// typedef was replaced w/ Using keyword (works better w/ templates)
// using str_t = std::string;

enum Day {sunday = 0, monday = 1, tuesday = 2, wednesday = 3, thursday = 4, 
        friday = 5, saturday = 6};  // Can assign diff ints...

enum Flavor {vanilla, chocolate, strawberry, matcha};  // Can also not explicitly assign ints... default is =0,=1,=2...

// Function Initializations
void happyBday(std::string name, int age);  // Just declare here, more readable if main() func 1st...
void happyBday();
double getTotal(double prices[], int size);
int linearSearchArray(int numbers[], int size, int num_to_find);
void bubbleSort(int arr[], int size);
void printArrayInt(int arr[], int size);
void printArrayStr(std::string arr[], int size);
void fillArray(std::string arr[], int size);

// Tic Tac Toe
void boardTicTacToe(char *spaces);
void playerMove(char *spaces, char player);
void computerMove(char *spaces, char computer);
bool checkWinner(char *spaces, char player, char computer);
// bool checkTie(char *spaces);

void walk(int steps);
int fact(int terms);
int fib(int terms);

template <typename T>
void printArray(T arr[], int size) {
    std::cout << "[";
    for (int i=0; i<size; i++) {
        if (i == size-1) {
            std::cout << arr[i] << "]\n";
            break;
        }
        std::cout << arr[i] << ", ";
    }
}

template <typename T, typename U>
auto max(T x, U y) {  // T is convention, auto...
    return (x>y) ? x : y;
}

// struct Student{
//     std::string name;
//     double gpa;
//     bool is_enrolled;
//     bool is_pretty = true;  // W/ default value
// };

// void printStudent(Student student);

// OOP
class Human {
    public:
        std::string name;
        int age;
        std::string occupation;

        void eat() {
            std::cout << name << " is eating...\n";
        }
        void drink() {
            std::cout << name << " is drinking...\n";
        }
};

class Student {
    public: 
        std::string name;
        int age;
        double gpa;

    Student(std::string name, int age, double x) {
        this->name = name;
        this->age = age;
        gpa = x;  // If param name != attribute name
    }
};

class Pizza {
    public:
        std::string topping1;
        std::string topping2;
    
    Pizza(std::string topping1) {
        this->topping1 = topping1;
    }
    Pizza(std::string topping1, std::string topping2) {
        this->topping1 = topping1;
        this->topping2 = topping2;
    }
    Pizza() {
    }

};

class Stove {
    private:
        int temperature = 0;
    public:
        Stove(int temperature) {
            setTemperature(temperature);  // Invoke setter in constructor...
        }
        int getTemperature() {
            return temperature;
        }
        void setTemperature(int temperature) {
            if (temperature < 0) {  // Checks...
                this->temperature = 0;
            }
            else if (temperature >= 10) {
                this->temperature = 10;
            }
            else {
                this->temperature = temperature;
            }
        }
};

class Animal {
    public:
        bool alive = true;
        void eat() {
            std::cout << "This animal is eating...\n";
        }
};

class Dog : public Animal {
    public:
        void bark() {
            std::cout << "Bark! Bark!\n";
        }
        void eat() {
            std::cout << "Biting bone...\n";
        }
};

class Cat : public Animal {
    public:
        void meow() {
            std::cout << "Meow! Meow!\n";
        }
};

int main() {
    /*
    // Basics
    int x = 5.5;  // Declaration vs Assignment
    double y = 6.0;  // Like float?
    double sum = x + y;  // Type coercion...
    // char, bool, string
    std::string name = "Ally";

    std::cout << "I like programming!" << std::endl;  // endl will flush output buffer?
    std::cout << "Like really!" << '\n';  // \n is better performance-wise, '' can only take 4Bytes?
    std::cout << sum << '\n';
    std::cout << name + '\n';  // + is for string concatenation only...
    // std::cout << "You are " + sum + " years old" + '\n';  // So error...
    std::cout << "You are " << sum << " years old" << '\n';
    */

    /*
    // const (prevents modification, read-only)
    const double PI = 3.14159;
    // PI = 420.69;  // So error...
    double radius = 10;
    double circumference = 2 * PI * radius;
    std::cout << circumference << " cm\n";

    // int x = 0;
    using namespace first;
    std::cout << x;

    // typedef- reserved keyword to create additional name/alias for another dtype
        // New id for existing type, helps w/ readability & reduce typos
    // type aliases- 
    // pairlist_t pairlist;
    str_t name1 = "Lex";
    std::cout << name1 << '\n';
    */

    /*
    // +, -, *, /, %
    // num+=1, num++ (Increment operator), num-=2, num--...
    // PEMDAS, care...

    // Implicit (automatic) vs Explicit (precede value w/ new dtype) type conversion
    double x = (int) 3.14;  // Explicit
    std::cout << x << '\n';
    char y = 100;
    std::cout << y << '\n';  // Decimal to char, ASCII table
    std::cout << (char) 102 << '\n';

    int correct = 8, questions = 10;
    double score = (double) correct / (double) questions * 100;  // 0 if no explicit conversion, (double) <denom> is enough...
    std::cout << score << '%' << '\n';

    // Input
    std::string name;
    int age;

    std::cout<< "What's your age?: ";
    std::cin >> age;
    std::cout << "What's your full name?: ";
    // std::cin >> name;
    std::getline(std::cin >> std::ws, name);  // If there's space in input, care... >> std::ws (extract white space...)
    std::cout << "Hello " << name << '\n';
    std::cout << "You are " << age << " years old" << "\n";

    // Other operators, cmath...
    double x=3, y=4, z;
    z = std::max(x,y);  // There's also std::min()
    z = pow(2,3);  // There's also sqrt(), abs(), round(), ceil(), floor(), 
    std::cout << z << '\n';
    */

    /*
    // Conditionals- if-else if-else, switch-case-default ('break' to avoid fall-through...)
    
    // Ternary Operators- replacement of if-else, <condition> ? <expression1> : <expression2>;, if true then expression1...
    int grade = 74;
    std::string remark;
    grade >= 75 ? remark = "You passed!" : remark = "You failed!";
    std::cout << remark << '\n';

    // Logical Operators- &&, ||, !

    // String Methods- concatenate..., 
    std::string name;
    std::cout << "Enter you name: ";
    std::getline(std::cin >> std::ws, name);

    std::cout << name.length();  // Includes spaces
    // There's also .empty(), .clear(), .append(<string>), .insert(<idx>, <string>), 
    // .at(<int>) (returns n-th char), .find(<string>) (returns idx of 1st <string> found...)
    // .erase(<idx1>, <idx2>) (removes elements from idx1 to idx2 - 1)

    // Iterations- while (care inf loop), do-while, for (usually for iterables?), nested (i>j>k...) loops
    // break & continue (skip current iteration)
    for (int i=1; i<=20; i++) {
        if (i == 13) {  // Unlucky #
            continue;  // Just skip 13
        }
        std::cout << i << '\n';
    }

    // Pseudo-random #s (not truly random but close)
    srand(time(NULL));  // Programmers typically use current calendar time as seed
    double num1 = (rand() % 6) + 1;  // Default is 0-32,767, do this for 1-6..., just ints...
    int num2 = (rand() % 6) + 1;
    std::cout << num1 << '\n';
    std::cout << num2 << '\n';
    // Can use this concept for randomizing strings too... add switch-case...

    // (User-defined) Functions- return, overload (same func name but diff params)
    // Variable scope- Local vs Global... overwrite..., use :: (SRO) to use Global instead...
    std::string name1 = "Ally", name2 = "Lex";
    int age1 = 26, age2 = 24;
    happyBday(name1, age1);  // Func call / invoke func
    happyBday(name2, age2);
    happyBday("Katana", 24);
    happyBday();

    std::cin.clear();  // To reset any err flags...
    fflush(stdin);  // ?
    */

    /*
    // Arrays- iterate over array, foreach loop (less flexible but simpler...)
    // Pass array to func (*pointer... since when func receives an array as arg is it decays to pointer... no longer knows array size..., 
        // for now is we also pass array size), 
    std::string cars[] = {"Corvette", "Toyota", "Honda"};  // Plural, must be same dtype
    std::string phones[3];  // Must put #elements if we'll add the elements later...
    cars[0] = "Lamborghini";
    std::cout << cars[0] << '\n';  // Will ouput memory addr if just car...

    // sizeof()- size in bytes of a variable, dtype, class, obj's...
    // double (8Bytes), char (1Byte), bool (1Byte), string (32Bytes... holds addr to where txt is located... and the addr is 32Bytes), 
    // arrays sizeof() depends in #elements..., to know #elements then sizeof(<array>)/sizeof(<1 element> or <dtype>)

    cars[3] = "Mitsubishi";  // This will cause weird behaviors (since sizeof(cars) must be 3...), use vectors for dynamic...
    std::cout << cars[0] << cars[1] << cars[2] << cars[3] << '\n';
    std::cout << sizeof(cars)/sizeof(std::string);
    // Segmentation fault- 

    for (std::string car : cars) {
        std::cout << car << '\n';
    }

    double prices[] = {49.99, 15.05, 75, 9.99};
    std::cout << getTotal(prices, sizeof(prices)/sizeof(double)) << '\n';

    // Arrays- search, sort, fill(), multidimensional arrays
    int numbers[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    int num_to_find;
    std::cout << "Enter number to find: " << '\n';
    std::cin >> num_to_find;
    int index = linearSearchArray(numbers, sizeof(numbers)/sizeof(int), num_to_find);
    std::cout << "The index is: " << index << '\n';

    int arr[] = {1,3,2,10,9,6,7,8,5,4}, size = 10;
    int arr[] = {10,1,9,2,8,3,7,4,6,5}, size = 10;
    // // Randomize array
    // // Bubble sort
    bubbleSort(arr, size);
    printArray(arr, size);

    std::string arr[10];
    int size = sizeof(arr)/sizeof(std::string);
    int midpoint = size / 2;
    fill(arr, arr + midpoint, "Ally");  // fill(<mem addr start>, <mem addr end>, <value>)
    fill(arr + midpoint, arr + size, "Lex");
    printArrayStr(arr, sizeof(arr)/sizeof(std::string));

    std::string names[5];
    int size = sizeof(names)/sizeof(std::string);
    // int idx = 0;
    fillArray(names, size);
    printArrayStr(names, size);

    std::string categories[][3] = {{"Gord", "Nana", "Valir"},  // [<row>][<col>]
                                {"Hanabi", "Lesley", "Irithel"}, 
                                {"Mathilda", "Minotaur", "Hylos"}, 
                                {"Ling", "Joy", "Hayabusa"}};
    // std::cout << categories[2][0] << '\n';

    int row_size = sizeof(categories)/sizeof(categories[0]);
    int col_size = sizeof(categories[0])/sizeof(categories[0][0]);

    std::cout << row_size << "x" << col_size << '\n';

    for (int i=0; i<row_size; i++) {
        for (int j=0; j<col_size; j++) {
            std::cout << categories[i][j] << " ";
        }
    }
    */

    /*
    // Mem addr's (& is addr-of operator), pass by value (e.g. pass variable to func... add & to pass by ref...) vs ref (more useful?), 
    // const params (useful for refs&pointers... so if we pass by ref then we won't accidentally change it..., in func def...), 
        // const *<pointer>... nothing can change addr of where pointer points
    // Pointers (* is dereference operator, variable that stores mem addr of another variable)
        // Sometimes it's easier to work w/ addr, instead of handing out elements is you tell them where the elements are...
    // Null value, nullptr (if pointer holds null then it's not pointing at anything, to determine if addr was successfully assigned to pointer)
    std::string name = "JD Castor";
    std::cout << &name << '\n';  // 1 use of dtype is to allocate enough memory...

    // Check if credit card # is valid (via Luhn algo)

    std::string *pName = &name;
    std::cout << pName << '\n';  // Yields addr
    std::cout << *pName << '\n';  // Yields value (dereferenced)

    std::string foods[] = {"ramen", "fried rice", "steak", "burger", "sashimi"};
    std::string *pfoods = foods;  // Care &... array is already an addr
    std::cout << *pfoods << '\n';  // Deref to 1st element

    // int *pointer;
    int *pointer = nullptr;  // Better practice, else we don't know where it's pointing
    int x = 123;
    pointer = &x;
    
    if (pointer == nullptr) { // Better practice is to check of pointer is nullptr...
        std::cout << "addr not assigned\n";
    }
    else {
        std::cout << "addr is assigned\n";
        std::cout << *pointer;  // Unsafe to deref nullptr
    }

    // Tic Tac Toe
    char board[9];
    int size = sizeof(board)/sizeof(char);
    std::fill(board, board+size, ' ');
    char player = 'X';
    char computer = 'O';
    bool playing = true;

    // boardTicTacToe(board);  // When we pass array to func is it decays to pointer already...

    while(playing) {
        playerMove(board, player);
        boardTicTacToe(board);
        if (checkWinner(board, player, computer)) {
            playing=false;
            break;
        }
        computerMove(board, computer);
        boardTicTacToe(board);
        if (checkWinner(board, player, computer)) {
            playing=false;
            break;
        }
    }
    */

    /*
    // Dynamic memory- mem allocated after prog is already compiled&running, 
        // new operator to allocate mem in heap than stack
    int *pNum = nullptr;  // Good practice to initialize pointer w/ NULL/nullptr first?
    pNum = new int;  // new returns addr...
    *pNum = 67;  // assign value

    std::cout << pNum << ' ' << *pNum << '\n';

    // Good practice to also use delete operator when using new... when we no longer use the mem space...
    delete pNum;  // If not then we may cause mem leak...

    char *pGrades = NULL;
    int size;

    std::cout << "How many grades to enter in?: ";
    std::cin >> size;
    pGrades = new char[size];

    for (int i=0; i<size; i++) {
        std::cout << "Enter grade #" << i+1 << ": ";
        std::cin >> pGrades[i];
    }

    for (int i=0; i<size; i++) {
        std::cout << pGrades[i] << ' ';
    }

    delete[] pGrades;

    // Recursion- useful for sorting&searching algs... tree navig..., cleaner code but uses more mem & slower
    walk(10);
    std::cout << fact(5) << '\n';
    std::cout << fib(5) << '\n';

    // Func templates- describes what a func looks like, to generate as many overload funcs, each using diff dtypes
    std::cout << max(2, 2.1) << '\n';
    int arr[] = {1,3,5,2,4,6};
    printArray(arr, sizeof(arr)/sizeof(int));

    // Structs- groups related variables under 1 name, can contain many diff dtypes, 
        // Members (variables in struc), . (Class Member Access Operator), 
        // Like array but can have multiple dtypes...
    Student Ally;
    Ally.name = "Ally";
    Ally.gpa = 3.2;
    Ally.is_enrolled = true;
    // std::cout << Ally.name << ' ' << Ally.gpa << ' ' << Ally.is_enrolled << '\n';

    Student Lex;
    Lex.name = "Lex";
    Lex.gpa = 2.1;
    Lex.is_enrolled = true;
    // std::cout << Lex.name << ' ' << Lex.gpa << ' ' << Lex.is_pretty << '\n';
        // Pass structs as args
    printStudent(Ally);  // Structs are passed by value, pass by ref (&) if we wanna change...
    printStudent(Lex);

    // Enum- user-defined dtype, paired named-int constants, if we have set of potential options
        // vs switch-case (can't use string here, use Enums...)
    Day today = monday;  // Like dtype..., can now use in switch-case since there's associated int
        // Can only take 1 value of the set...
    switch (today) {
        case sunday: std::cout << "Today is sunday. \n";
                    break;
        case 1: std::cout << "Today is monday. \n";  // Can also use the associated int
                    break;
        case tuesday: std::cout << "Today is tuesday. \n";  // And so on...
    }
    */

    /*
    // OOP
    Human Ally;
    Ally.name = "Ally";
    Ally.age = 26;
    Ally.occupation = "Student";
    std::cout << Ally.name << ' ' << Ally.age << ' ' << Ally.occupation << '\n';
    Ally.eat();
    Ally.drink();

    // Constructors- special method that is automatically called when obj is instantiated, 
        // For assigning values to attributes as args
        Student student1("Ally", 26, 3.2);
        std::cout << student1.name << ' ' << student1.age << ' ' << student1.gpa << '\n';
    // Constructor overloading- multiple constructors w/ same name but diff params, 
        // For varying args when instantiating obj
    Pizza pepperoni("pepperoni");
    Pizza hawaiian("ham", "pineapple");
    Pizza pizza;
    std::cout << pizza.topping1;
    std::cout << pepperoni.topping1 << ' ' << pepperoni.topping2 << '\n';
    std::cout << hawaiian.topping1 << ' ' << hawaiian.topping2 << '\n';

    // Getters (func that makes private attr readable) & Setters (func that makes private attr writeable)
        // For Abstraction, perform additional checks&logics...
    Stove stove(5);
    std::cout << stove.getTemperature() << '\n';
    stove.setTemperature(37);
    std::cout << stove.getTemperature();
    */

    // Inheritance- class can receive attributes&methods from another class, 
        // Children can inherit from Parent, helps reuse similar code w/in multiple classes
    Dog doggy;
    std::cout << doggy.alive << '\n';
    doggy.eat();
    doggy.bark();
    Cat catty;
    catty.eat();
    catty.meow();

    return 0;
}

void happyBday(std::string name, int age) {  // void- no return
    for (int i=1; i<=4; i++) {
        if (i==3) {
            std::cout << "Happy bday, " << name << ".\n";
            continue;
        }
        std::cout << "Happy bday to you.\n";
    }
    std::cout << "You are now " << age << " years old!\n";
    std::cout << '\n';
}

void happyBday() {
    std::cout << "I don't know you but happy birthday!\n";
}

double getTotal(double prices[], int size) {  // Func signature
    double total = 0.0;
    for (int i=0; i<size; i++) {
        total += prices[i];
    }
    return total;
}

int linearSearchArray(int numbers[], int size, int num_to_find) {
    for (int i=0; i<size; i++) {
        if (numbers[i] == num_to_find) {
            return i;
        }
    }
    return -1;  // Acts as Sentinel value (not found...)
}

void arrayRandomizer() {
}

void bubbleSort(int arr[], int size) {
    for (int i=0; i<(size-1); i++) {
        int temp;
        for (int j=0; j<(size-i-1); j++) {
            if (arr[j] > arr[j+1]) {  // < Then descending
                // arr[j], arr[j+1] = arr[j+1], arr[j];
                // arr[j+1] = arr[j];
                // std::swap(arr[j], arr[j+1]);
                temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
        }
        // std::cout << arr[size-i-1] << '\n';
    }
}

void printArrayInt(int arr[], int size) {
    std::cout << "[";
    for (int i=0; i<size; i++) {
        if (i == size-1) {
            std::cout << arr[i] << "]\n";
            break;
        }
        std::cout << arr[i] << ", ";
    }
}

void printArrayStr(std::string arr[], int size) {
    std::cout << "[";
    for (int i=0; !arr[i].empty(); i++) {
        if (i == size-1 || arr[i+1].empty()) {
            std::cout << arr[i] << "]\n";
            break;
        }
        std::cout << arr[i] << ", ";
    }
}

void fillArray(std::string arr[], int size) {
    std::string arr_element;

    for (int i=0; i<size; i++) {
        std::cout << "Enter string #" << i + 1 << " or press 'q' to quit: \n";
        std::getline(std::cin >> std::ws, arr_element);
        if (arr_element == "q") {
            break;
        }
        else {
            arr[i] = arr_element;
        }
    }
    // else {
    //     std::cout << "Sorry, array is full. \n";
    // }
}

// Tic Tac Toe
void boardTicTacToe(char *spaces) {
    std::cout << spaces[0] << spaces[1] << spaces[2] << '\n';
    std::cout << spaces[3] << spaces[4] << spaces[5] << '\n';
    std::cout << spaces[6] << spaces[7] << spaces[8] << '\n';
}
void playerMove(char *spaces, char player) {
    int temp;
    do {
        std::cout << "Which spot (0-8) do you wanna play?: \n";
        std::cin >> temp;
        if (spaces[temp] != ' ') {
            std::cout << "This spot is taken, choose another. \n";
            continue;
        }
        else if (temp < 0 || temp > 8) {
            std::cout << "Invalid input, choose 0-8. \n";
            continue;
        }
        spaces[temp] = player;
        break;
    } while (1);
    
}
void computerMove(char *spaces, char computer) {
    int num_rand;
    srand(time(NULL));
    do {
        num_rand = rand() % 9;
        if (spaces[num_rand] != ' ') {
            continue;
        }
        spaces[num_rand] = computer;
        break;
    } while (1);
}
bool checkWinner(char *spaces, char player, char computer) {
    // switch-case will be more complicated for beginners
    if (spaces[0] != ' ' && spaces[0] == spaces[1] && spaces[0] == spaces[2]) {
        spaces[0] == player ? std::cout << "Player wins!\n" : std::cout << "Computer wins!\n";
    }
    else if (spaces[3] != ' ' && spaces[3] == spaces[4] && spaces[3] == spaces[5]) {
        spaces[3] == player ? std::cout << "Player wins!\n" : std::cout << "Computer wins!\n";
    }
    else if (spaces[6] != ' ' && spaces[6] == spaces[7] && spaces[6] == spaces[8]) {
        spaces[6] == player ? std::cout << "Player wins!\n" : std::cout << "Computer wins!\n";
    }
    else if (spaces[0] != ' ' && spaces[0] == spaces[3] && spaces[0] == spaces[6]) {
        spaces[0] == player ? std::cout << "Player wins!\n" : std::cout << "Computer wins!\n";
    }
    else if (spaces[1] != ' ' && spaces[1] == spaces[4] && spaces[1] == spaces[7]) {
        spaces[1] == player ? std::cout << "Player wins!\n" : std::cout << "Computer wins!\n";
    }
    else if (spaces[2] != ' ' && spaces[2] == spaces[5] && spaces[2] == spaces[8]) {
        spaces[2] == player ? std::cout << "Player wins!\n" : std::cout << "Computer wins!\n";
    }
    else if (spaces[0] != ' ' && spaces[0] == spaces[4] && spaces[0] == spaces[8]) {
        spaces[0] == player ? std::cout << "Player wins!\n" : std::cout << "Computer wins!\n";
    }
    else if (spaces[2] != ' ' && spaces[2] == spaces[4] && spaces[2] == spaces[6]) {
        spaces[2] == player ? std::cout << "Player wins!\n" : std::cout << "Computer wins!\n";
    }
    else {
        // See if tie
        for (int i=0; i<9; i++) {
            if (spaces[i] == ' ') {
                break;
            }
            else if (i == 8) {
                std::cout << "It's a tie! \n";
                return 1;
            }
        }
        return 0;
    }
    return 1;
}
// bool checkTie(char *spaces) {
//     return 0;
// }

void walk(int steps) {
    if (steps == 0) {  // Base case
        return;
    }
    walk(steps-1);
    std::cout << "Step " << steps << "... ";  // Swap last 2 lines for decreasing...
    // Stack overflow...
}

int fact(int terms) {
    if (terms == 0 || terms == 1) {
        return 1;
    }
    else {
        return terms * fact(terms-1);
    }
}

int fib(int terms) {
    if (terms == 0 || terms == 1) {
        return 1;
    }
    else {
        return (fib(terms-1) + fib(terms-2));
    }
}

// void printStudent(Student student) {
//     std::cout << student.name << ' ' << student.gpa << ' ' << student.is_enrolled << ' ' << student.is_pretty << '\n'; 
// }