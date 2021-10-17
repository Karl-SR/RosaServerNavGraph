#include "navgraphgenerator.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <stack>
#include <unordered_map>

lineIntersectLevelFunc lineIntersectLevel;

bool Vector::isInCuboid(const Vector& lower, const Vector& upper) const {
	return x >= lower.x && x <= upper.x && y >= lower.y && y <= upper.y &&
	       z >= lower.z && z <= upper.z;
}

static const std::tuple<float, float> isTouchingGroundCheckpoints[] = {
    {0.f, 0.f},
    {0.251f, 0.251f},
    {0.251f, -0.251f},
    {-0.251f, 0.251f},
    {-0.251f, -0.251f}};

static const std::tuple<float, float, float> lineOfSightCheckpoints[] = {
    {0.f, 0.f, 0.f},         {0.01f, 0.01f, 0.01f},   {0.01f, 0.01f, -0.01f},
    {-0.01f, 0.01f, 0.01f},  {-0.01f, 0.01f, -0.01f}, {0.01f, -0.01f, 0.01f},
    {0.01f, -0.01f, -0.01f}, {-0.01f, -0.01f, 0.01f}, {-0.01f, -0.01f, -0.01f},
};

static inline bool doVoxelsHaveLineOfSight(Vector& posA, Vector posB) {
	float deltaX = posB.x - posA.x;
	float deltaY = posB.y - posA.y;
	float deltaZ = posB.z - posA.z;

	posB.x = posA.x + (deltaX * 1.1f);
	posB.y = posA.y + (deltaY * 1.1f);
	posB.z = posA.z + (deltaZ * 1.1f);

	for (const auto [x, y, z] : lineOfSightCheckpoints) {
		Vector intersectA{posA.x + x, posA.y + y, posA.z + z};
		Vector intersectB{posB.x + x, posB.y + y, posB.z + z};

		if (lineIntersectLevel(&intersectA, &intersectB, false)) {
			return false;
		}
	}

	return true;
}

bool NodePoint::isTouchingGround() const {
	auto pos = toWorld();
	for (const auto [x, z] : isTouchingGroundCheckpoints) {
		Vector posA{pos.x + x, pos.y, pos.z + z};
		Vector posB{posA.x, posA.y - 1.5f - 0.002f, posA.z};
		if (!lineIntersectLevel(&posA, &posB, true)) {
			return false;
		}
	}
	return true;
}

bool NodePoint::isTouchingGroundWithLineOfSightFrom(Vector& otherPos,
                                                    Vector& otherPosTop) const {
	if (!isTouchingGround()) {
		return false;
	}

	auto pos = toWorld();

	if (!doVoxelsHaveLineOfSight(otherPos, pos)) {
		return false;
	}

	auto posTop = toWorldTop();

	if (!doVoxelsHaveLineOfSight(otherPosTop, posTop)) {
		return false;
	}

	return true;
}

namespace std {
template <>
struct hash<NodePoint> {
	std::size_t operator()(const NodePoint& key) const {
		return hash<int>()(key.x) + 17 * hash<int>()(key.y) +
		       37 * hash<int>()(key.z);
	}
};
}  // namespace std

static const std::tuple<int, int> neighbours[] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

void NavGraphGenerator::generate(int originX, int originY, int originZ,
                                 sol::this_state s) {
	sol::state_view lua(s);
	sol::protected_function print = lua["print"];

	unsigned int rootId = 0;
	std::vector<Node> nodes;
	{
		NodePoint origin{originX, originY, originZ};
		nodes.emplace_back(origin);
	}

	std::unordered_map<NodePoint, unsigned int> nodeIdByPoint;
	nodeIdByPoint.insert({nodes[0].point, 0});

	std::stack<unsigned int> backtrackIds;
	backtrackIds.push(0);

	while (true) {
		if (rootId >= 3'000'000) {
			print("Halted");
			break;
		}

		bool rootChanged = false;
		Node& root = nodes[rootId];

		for (const auto [neighbourX, neighbourZ] : neighbours) {
			NodePoint tryPoint{root.point.x + neighbourX, root.point.y,
			                   root.point.z + neighbourZ};

			Vector rootPos = root.point.toWorld();
			Vector rootPosTop = root.point.toWorldTop();

			bool isConnected =
			    tryPoint.isTouchingGroundWithLineOfSightFrom(rootPos, rootPosTop);
			if (!isConnected) {
				tryPoint.y += 1;
				isConnected =
				    tryPoint.isTouchingGroundWithLineOfSightFrom(rootPos, rootPosTop);
				if (!isConnected) {
					tryPoint.y -= 2;
					isConnected =
					    tryPoint.isTouchingGroundWithLineOfSightFrom(rootPos, rootPosTop);
				}
			}

			if (isConnected) {
				auto search = nodeIdByPoint.find(tryPoint);
				if (search != nodeIdByPoint.end()) {
					root.linkIds.insert(search->second);
				} else {
					nodes.emplace_back(Node(tryPoint));
					unsigned int insertedIndex = nodes.size() - 1;
					nodeIdByPoint.insert({tryPoint, insertedIndex});

					if (insertedIndex % 10'000 == 0) {
						std::ostringstream stream;
						stream << "Created node " << insertedIndex << " at [" << tryPoint.x
						       << ", " << tryPoint.y << ", " << tryPoint.z
						       << "], #stack = " << backtrackIds.size();
						print(stream.str());
					}

					nodes[rootId].linkIds.insert(insertedIndex);
					backtrackIds.push(rootId);
					rootId = insertedIndex;
					rootChanged = true;
					break;
				}
			}
		}

		if (!rootChanged) {
			if (backtrackIds.empty()) {
				print("Stack clear, we are finished");
				break;
			} else {
				rootId = backtrackIds.top();
				backtrackIds.pop();
			}
		}
	}

	{
		std::ostringstream stream;
		stream << nodes.size() << " nodes generated";
		print(stream.str());
	}

	generatedNodes = nodes;
}

void NavGraphGenerator::addStreetCuboid(Vector* lower, Vector* upper) {
	streetCuboids.emplace_back(std::make_tuple(*lower, *upper));
}

int NavGraphGenerator::getHighestStreetY() const {
	float highestY = 0;

	for (const auto [lower, upper] : streetCuboids) {
		if (upper.y > highestY) {
			highestY = upper.y;
		}
	}

	return static_cast<int>(std::floor(highestY));
}

void NavGraphGenerator::markStreetNodes() {
	const int highestStreetY = getHighestStreetY();

	for (auto& node : generatedNodes) {
		node.isOnStreet = false;

		if (node.point.y > highestStreetY) {
			continue;
		}

		auto pos = node.point.toWorld();

		for (const auto [lower, upper] : streetCuboids) {
			if (pos.isInCuboid(lower, upper)) {
				node.isOnStreet = true;
				break;
			}
		}
	}
}

void NavGraphGenerator::write(const std::string fileName) const {
	std::ofstream file(fileName, std::ios_base::binary);

	uint32_t numNodes = generatedNodes.size();
	file.write(reinterpret_cast<const char*>(&numNodes), sizeof(numNodes));

	for (auto& node : generatedNodes) {
		int16_t x = static_cast<int16_t>(node.point.x);
		int16_t y = static_cast<int16_t>(node.point.y);
		int16_t z = static_cast<int16_t>(node.point.z);
		uint8_t isOnStreet = node.isOnStreet;
		file.write(reinterpret_cast<const char*>(&x), sizeof(x));
		file.write(reinterpret_cast<const char*>(&y), sizeof(y));
		file.write(reinterpret_cast<const char*>(&z), sizeof(z));
		file.write(reinterpret_cast<const char*>(&isOnStreet), sizeof(isOnStreet));
		uint8_t numLinks = static_cast<uint8_t>(node.linkIds.size());
		file.write(reinterpret_cast<const char*>(&numLinks), sizeof(numLinks));
		for (uint32_t linkId : node.linkIds) {
			file.write(reinterpret_cast<const char*>(&linkId), sizeof(linkId));
		}
	}

	file.close();

	if (!file) {
		std::ostringstream stream;
		stream << "Failed to write to file: " << strerror(errno);
		throw std::runtime_error(stream.str());
	}
}