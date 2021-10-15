#include <GL/glut.h>
#include <iostream>

void keyboard(unsigned char key, int x, int y){
    std::cout << key << " : down" <<  std::endl;
}

void keyup(unsigned char key, int x, int y){
    std::cout << key << " : up" << std::endl;
}

int main(int argc, char *argv[])
{
  glutInit(&argc, argv);
  glutSetKeyRepeat(GLUT_KEY_REPEAT_OFF);
  glutCreateWindow(argv[0]);
  glutKeyboardFunc(keyboard);
  glutKeyboardUpFunc(keyup);
  glutMainLoop();
  return 0;
}