#include "App.h"
#include <cstdlib>

// Entry point. Creates the app, initializes it, and runs the main loop.
int main(int argc, char* argv[])
{
    App app;


    if (!app.Init())
        return EXIT_FAILURE;

    app.Run();

    return EXIT_SUCCESS;
}
