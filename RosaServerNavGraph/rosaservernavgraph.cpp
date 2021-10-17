#include "sol/sol.hpp"

static sol::table openLibrary(sol::this_state L) {
	sol::state_view lua(L);
	sol::table library = lua.create_table();
	return library;
}

extern "C" __attribute__((visibility("default"))) int
luaopen_librosaservernavgraph(lua_State* L) {
	return sol::stack::call_lua(L, 1, openLibrary);
}