# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
      1. Take in a Vector2D object, calculate the magnitude of the vector using a standard math algorithm, and return another 
      Vector2D object where each component of the object is the component of the original object divided by the magnitude.
      2. Take in a Vector2D object, calculate the magnitude of the vector using a separate helper function, and return another 
      Vector2D object where each component of the object is the component of the original object divided by the magnitude.
      3. Take in two doubles that are the two vector components and return a Vector2D object in the same form of calculation
      as suggestion 1.

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
      1. This method is the best because according to ISO C++ it should take it the most clear variables (in this case, a
      Vector2D object) and use the simplest form to calculate. While this does create another variable in the middle, it is
      clear what the function should be doing.
      2. This function is not ideal because it violates the rule of creating unneccesary operations when standard operations
      exist. Since the function ignores the standard math library, it complicates things with a helper function losing clarity
      on intent.
      3. This function does the math efficiently, but it does take in the vectors as pieces which makes the input variables
      confusing since the intent is unclear. Two doubles is less clear than one Vector2D object when trying to see what the
      code is doing.

   - Which of the methods would you implement and why?
      - I would implement the first method because it is the clearest on inputs, execution, and it follows the C++ Core guidelines.

2. What is the difference between a class and a struct in C++?
   - The main difference between a struct and a class is that a class is default private where as a struct is default public.


3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
   - Vector2D is a set of two values (an x and y) meaning that it is a relatively simple data type. Structs are best used for
   organizing data, and classes often alert the user of an invariant. In this case, Transform2D has operations that are always
   true, therefore, a class is a better option.

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
   - We mark things as explicit because we don't want the things taken in to cause inheritance issues later in the code.
   By marking the constructor as explicit, the default inheritance of C++ does not apply. Not all functions can be explicit
   though beecause explicit copy/move constructors make passing and returning by value difficult.


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)] in your answer
   - This is a point on mutability. Transform2D::inv() creates a new transform that does not modify the input transform, so it keeps the values constant where as Transform2D::operator*=() will modify the object throughout the funciton meaning that the input gets changed and output.
