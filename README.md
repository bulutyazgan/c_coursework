Ongoing coursework assignment for UCL Programming Principles

## Project Structure

- **main.c** - Main program loop and drawing functionalities
- **robot.h/robot.c** - Core and advanced robot functionalities (movement, sensors, corner detection)
- **maps.h/maps.c** - Test obstacle course configurations (7 test courses)
- **path_planning.h/path_planning.c** - Coverage planning algorithms (A*, heuristics, coverage search)
- **graphics.h/graphics.c** - Graphics library (provided, do not modify)

## Building and Running

### Compile
```bash
gcc -o main main.c robot.c maps.c path_planning.c graphics.c
```

### Run
```bash
./main | java -jar drawapp-4.5.jar
```

### Shortcut
```bash
gcc -o main main.c robot.c maps.c path_planning.c graphics.c && ./main | java -jar drawapp-4.5.jar
```

### Test Courses
Change the test course by modifying `TEST_COURSE` constant in main.c (line 13):
```c
#define TEST_COURSE 3  // Select test course (1-7)
```
Available courses: 1=Empty, 2=Irregular Shaped Arena, 3=Central Wall, 4=Maze, 5=Scattered, 6=U-Shape, 7=Circular