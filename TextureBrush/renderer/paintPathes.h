#pragma once

#include "../preHeader.h"

namespace TextureSynthesis
{

class CPixelBufferObject;
class CPaintPathes
{
public:
	static CPaintPathes* Instance();
	virtual ~CPaintPathes();

	void startNewPath();
	void addPointToPath(const ivec2 &newPos);
	void endPath();

	void extractTriangleIndexTexture(GLuint texId);
	void compute3dPath();

	float DistanceFromPointToPath(vec3 point, vector<ivec2> m_pathPointVec);
	void AddPointFunc(int TriNum, ivec3* pTriIndices, vector<ivec2> m_pathPointVec, vector<int> &newPathTriangleIdxVec, vector<int> &CN);

	const vector<ivec2>& getPathPointVec(){ return m_pathPointVec; }
	const vector<vector<ivec2> >& getPathVec(){ return m_pathVec; }
	const vector<vector<vec3> >& getPathVec3D(){ return m_pathVec3D; }

	const vector<int>& getZeroOrderPathIdxVec(){ return m_zeroOrderPathIdxVec; }
	const vector<int>& getFirstOrderPathIdxVec(){ return m_firstOrderPathIdxVec; }

protected:
	CPaintPathes();

	void collectVertexVectors();
	void calculateEquidisLineSegments();
	void assignLocalTexcoords();

private:
	vector<ivec2> m_pathPointVec;
	vector<vector<ivec2> > m_pathVec;

	vector<vector<uint> > m_triIdxVec;
	vector<vector<vec3> > m_pathVec3D;

	vector<int> m_zeroOrderPathIdxVec;
	vector<int> m_firstOrderPathIdxVec;

	CPixelBufferObject* m_pPBO;
	uint* m_pTriangleIdxData;
};

}