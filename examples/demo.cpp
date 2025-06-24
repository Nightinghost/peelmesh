#include <peelmesh/window.hpp>
#include <peelmesh/utility.hpp>

int main()
{

    auto resources = GetProgramDirPath() / "resources";

    peelmesh::DemoApplication app{resources.generic_string()};
    app.Run();

    return 0;
}