#pragma once

#include "../preHeader.h"
#include "GW_GeodesicMesh.h"

namespace TextureSynthesis
{

class CTriangleMesh;
class GW::GW_GeodesicMesh;

class CGeodesicMesh
{
public:
	CGeodesicMesh(CTriangleMesh *pTriMesh);
	virtual ~CGeodesicMesh();

	void setupGeodesicMesh();
	void resetGeoMesh();

	void addSeed(int triIdx);
	void addSeeds(const vector<int>& seedTriIdxVec);
	
	void computeGeodesics(float *pDis);
	void refinePath(int startIdx);
	void computePath(int startIdx, int endIdx);
	void getVertexPos(int idx, vec3* buff);

	float getStopDistance(){ return m_stopDistance; }
	const vector<int>& getZeroOrderPathIdxVec(){ return m_zeroOrderPathIdxVec; }
	const vector<int>& getFirstOrderPathIdxVec(){ return m_firstOrderPathIdxVec; }

private:
	float m_stopDistance;
	CTriangleMesh *m_pTriMesh;
	GW::GW_GeodesicMesh *m_pGeoMesh;

	int m_interOrder;
	vector<int> m_zeroOrderPathIdxVec;
	vector<int> m_firstOrderPathIdxVec;
	vector<vec3> m_pathPointVec;

	float m_pathLength;
};

}