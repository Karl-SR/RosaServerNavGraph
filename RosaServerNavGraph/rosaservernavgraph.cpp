#include "navgraphgenerator.h"
#include "sol/sol.hpp"

static sol::table openLibrary(sol::this_state L) {
	sol::state_view lua(L);

	{
		uintptr_t baseAddress = lua["memory"]["getBaseAddress"]();
		lineIntersectPos = (Vector*)(baseAddress + 0x569ef720);
		lineIntersectLevel = (lineIntersectLevelFunc)(baseAddress + 0x88cf0);
	}

	{
		auto meta = lua.new_usertype<NavGraphGenerator>("NavGraphGenerator");
		meta["generate"] = &NavGraphGenerator::generate;
		meta["addStreetCuboid"] = &NavGraphGenerator::addStreetCuboid;
		meta["markStreetNodes"] = &NavGraphGenerator::markStreetNodes;
		meta["write"] = &NavGraphGenerator::write;
	}

	sol::table library = lua.create_table();
	return library;
}

extern "C" __attribute__((visibility("default"))) int
luaopen_librosaservernavgraph(lua_State* L) {
	return sol::stack::call_lua(L, 1, openLibrary);
}