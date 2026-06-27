#include "Trans.h"

int main(int argc, char** argv)
{
	Trans node;
	if (!node.PramOpt(argc, argv))
	{
		return 0;
	}

	if (node.Init())
	{
		node.Run();
	}

    return 0;
}
