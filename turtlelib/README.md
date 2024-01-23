#Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.
Conceptual questions are answered using external sources cited in the citation.txt file, along with refernces to the C++ Core Guidelines found [here](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#main). 

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
        1. Do element-wise division of the vector by its magnitude in the main script
        2. Overload the division (or another operator) operator to perfrom element by element operation when dividing by a vector.
        3. Creating a new function that takes in a vector and returns the same normalized vector.
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
         1. **Pro:** This is easy to implement; **Con:** P.9: Don't waste time or space, this is a redundant operation that will be performed many times. 
         <!-- Begin Citation [2] -->
         2. **Pro:** "... you can exploit the intuition of the users of that class. This lets users program in the language of the problem domain rather than in the language of the machine." **Con:** Depending on implementation, this could require the creation of more than one function.
         <!-- End Citation [2] -->
         3. **Pro:** F.3: Keep functions short and simple, this function is basic and no more than a few lines. **Con:** this overwrite the pervious vector, which may be unfavorable in some cases.

   - Which of the methods would you implement and why?
         - I would pick the second method of overloading an opperator because it functionally takes up less space than creating a new function. Additionally, it makes me looker smarter and I like overloading operators.

2. What is the difference between a class and a struct in C++?:   <!-- Begin Citation [3] -->  
    - The main difference between a class and a struct in C++ is that a class defaults to private while a struct defaults to public.

<!-- End Citation [3] -->

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
   - *C.2: Use class if the class has an invariant; use struct if the data members can vary independently* Vector 2D is a struct because we want an individual to be able to modify elements of the strucutre directly. However, with the Transform2D class, we want to be able to control how the elements are modified. A rotations matrix has fundamental properties that need to me maintained. Allowing someone to edit specific elements of the matrix could cause the matrix to lose its properties. 
   - *C.9: Minimize exposure of members* "Reason Encapsulation. Information hiding. Minimize the chance of unintended access. This simplifies maintenance" 


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
   - Some constructors in Transform2D are explicit because we do not want to be able to control how elements of the class are modified. *C.46: By default, declare single-argument constructors explicit* Reason To avoid unintended conversions.


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
   - `Transform2D::inv()` is declared *const* because it does not modify the object.
   - `Transform2D::operator*=()` is not declared *const* because it modifies the object using the `*this` pointer. 
   - *Con.2: By default, make member functions const* A member function should be marked const unless it changes the object’s observable state. This gives a more precise statement of design intent, better readability, more errors caught by the compiler, and sometimes more optimization opportunities.
