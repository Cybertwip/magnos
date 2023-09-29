#pragma once
#include <axmol.h>

class Car : public ax::Node {
private:
	ax::Node* carBody;
	ax::Node* gearBox;
	std::vector<ax::Node*> gimbals;
	// Other member variables for acceleration, friction, etc.
	
public:
	Car();
	virtual ~Car();
	
	std::vector<ax::Node*> getGimbals() const;
	
	CREATE_FUNC(Car);

	
	// Define member functions for acceleration, friction, etc.
};

