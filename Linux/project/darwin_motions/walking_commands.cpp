#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <term.h>
#include <termios.h>
#include <libgen.h>
#include "cmd_process.h"
#include <python2.7/Python.h>

#define INI_FILE_PATH "../../Data/config.ini"

using namespace Robot;

LinuxCM730 linux_cm730("/dev/ttyUSB0");
CM730 cm730(&linux_cm730);

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void sighandler(int sig)
{
    struct termios term;
    tcgetattr( STDIN_FILENO, &term );
    term.c_lflag |= ICANON | ECHO;
    tcsetattr( STDIN_FILENO, TCSANOW, &term );

    exit(0);
}

void initWalking(){
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);

    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        printf("Fail to initialize Motion Manager!\n");
    }
    MotionManager::GetInstance()->LoadINISettings(ini);
    Walking::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    MotionManager::GetInstance()->SetEnable(true);
    printf("Walking initialized...\n");
}
    
void MOVE_X(float x){
    Walking::GetInstance()->X_MOVE_AMPLITUDE = x;
    Walking::GetInstance()->Start();
}

void stopWalking(){
    Walking::GetInstance()->Stop();
}

// PYTHON WRAPPING

static PyObject * initWalking_wrapper(PyObject * self)
{
    initWalking();
    Py_RETURN_NONE;
}

static PyObject * MOVE_X_wrapper(PyObject * self, PyObject * args){
    float x_val;

    if (!PyArg_ParseTuple(args, "f", &x_val)){
        return NULL;
    }

    // Call the actual function
    MOVE_X(x_val);

    Py_RETURN_NONE;
}

static PyObject * stopWalking_wrapper(PyObject * self){
    stopWalking();
    Py_RETURN_NONE;
}

static PyMethodDef WalkingMethods[] = {
    { "initWalking", (PyCFunction) initWalking_wrapper, METH_NOARGS, "Initialize walking module"},
    { "MOVE_X", (PyCFunction) MOVE_X_wrapper, METH_VARARGS, NULL},
    { "stopWalking", (PyCFunction) stopWalking_wrapper, METH_NOARGS, NULL},
    { NULL }
};

PyMODINIT_FUNC initwalking_module() {
    Py_InitModule3("walking_module", WalkingMethods, "meh");
}
