#include "App.h"
#include "CLI.h"
#include <cstdlib>
#include <string>

// Entry point. Creates the app, initializes it, and runs the main loop.
// Pass --cli (or -c) to run in command-line mode without a window.
int main(int argc, char* argv[])
{
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--cli" || std::string(argv[i]) == "-c") {
            CLI cli;
            cli.Run();
            return EXIT_SUCCESS;
        }
    }

    App app;

    if (!app.Init())
        return EXIT_FAILURE;

    app.Run();

    return EXIT_SUCCESS;
}
