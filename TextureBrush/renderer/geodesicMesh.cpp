#include "geodesicMesh.h"

#include "triangleMesh.h"
#include "GW_GeodesicMesh.h"
#include "GW_GeodesicPath.h"
#include "GW_Vertex.h"
#include "GW_Face.h"

using namespace TextureSynthesis;
using namespace GW;

// This callback is called every time a front vertex is visited to check
// if we should terminate marching.
static GW::GW_Bool FastMarchingStopCallback(
	GW::GW_GeodesicVertex& v, void *callbackData)
{
	// Stop if the vertex is farther than the distance stop criteria
	return (10000.0f <= v.GetDistance());
}


// This callback is invoked prior to adding new vertices to the front
static GW::GW_Bool FastMarchingVertexInsertionCallback(
	GW::GW_GeodesicVertex& v, GW::GW_Float distance, void *callbackData)
{
	return true;
}

// This callback is invoked to get the propagation weight at a given vertex.
// The default (if not specified) is a constant weight of 1 everywhere.
static GW::GW_Float FastMarchingPropagationWeightCallback(
	GW::GW_GeodesicVertex& v, void *callbackData)
{
	return 1.0f;
}

// This callback is invoked to get the propagation weight at a given vertex.
// Th result is no weight == 1
static inline GW_Float FastMarchingPropagationNoWeightCallback(
	GW_GeodesicVertex&, void *)
{
	return 1.0f;
}

CGeodesicMesh::CGeodesicMesh(CTriangleMesh *pTriMesh) : m_pTriMesh(pTriMesh), m_stopDistance(FLT_MAX), m_interOrder(1)
{
	setupGeodesicMesh();
}

CGeodesicMesh::~CGeodesicMesh()
{
	SAFE_DELETE(m_pGeoMesh);
}

void CGeodesicMesh::setupGeodesicMesh()
{
	// Get triangle mesh data
	int numVer = m_pTriMesh->getVerNum();
	int numTri = m_pTriMesh->getTriNum();

	vec3* pVertices = m_pTriMesh->getVertices();
	vec3* pNormals = m_pTriMesh->getNormals();
	ivec3* pTriIndices = m_pTriMesh->getTriIdx();
	
	// Setup geodesic mesh
	m_pGeoMesh = new GW_GeodesicMesh();
	
	// Copy vertex data
	m_pGeoMesh->SetNbrVertex(numVer);
	for (int verIdx = 0; verIdx < numVer; ++verIdx)
	{
		GW_GeodesicVertex &newGeoVertex = (GW::GW_GeodesicVertex &)m_pGeoMesh->CreateNewVertex();
		newGeoVertex.SetPosition(GW_Vector3D(pVertices[verIdx][0], pVertices[verIdx][1], pVertices[verIdx][2]));
		m_pGeoMesh->SetVertex(verIdx, &newGeoVertex);
	}

	// Copy triangle vertex index data
	m_pGeoMesh->SetNbrFace(numTri);
	for (int triIdx = 0; triIdx < numTri; ++triIdx)
	{
		GW_GeodesicFace& newGeoTriangle = (GW_GeodesicFace &) m_pGeoMesh->CreateNewFace();

		GW::GW_Vertex* a = m_pGeoMesh->GetVertex(pTriIndices[triIdx][0]);
		GW::GW_Vertex* b = m_pGeoMesh->GetVertex(pTriIndices[triIdx][1]);
		GW::GW_Vertex* c = m_pGeoMesh->GetVertex(pTriIndices[triIdx][2]);
		newGeoTriangle.SetVertex(*a, *b, *c);
		m_pGeoMesh->SetFace(triIdx, &newGeoTriangle);
	}

	// Configure geodesic mesh
	m_pGeoMesh->BuildConnectivity();
	m_pGeoMesh->RegisterForceStopCallbackFunction(FastMarchingStopCallback);
	m_pGeoMesh->RegisterVertexInsersionCallbackFunction(NULL);
	m_pGeoMesh->RegisterWeightCallbackFunction(FastMarchingPropagationNoWeightCallback);
	//m_pGeoMesh->ResetGeodesicMesh();
}

void CGeodesicMesh::resetGeoMesh()
{
	m_pGeoMesh->ResetGeodesicMesh();
}

// Fix : convert triangle indices to vertex indices
void CGeodesicMesh::addSeeds(const vector<int>& seedTriIdxVec)
{
	const int seedNum = seedTriIdxVec.size();

	for (int seedIdx = 0; seedIdx < seedNum; seedIdx++)
	{
		m_pGeoMesh->AddStartVertex(*((GW::GW_GeodesicVertex*)m_pGeoMesh->
			GetVertex((GW::GW_U32)(seedTriIdxVec[seedIdx]))));
	}
}

void CGeodesicMesh::addSeed(int triIdx)
{
	m_pGeoMesh->AddStartVertex(*((GW::GW_GeodesicVertex*)m_pGeoMesh->GetVertex((GW::GW_U32)(triIdx))));
}

void CGeodesicMesh::computeGeodesics(float *pDis)
{
	m_pGeoMesh->SetUpFastMarching();

	while (!m_pGeoMesh->PerformFastMarchingOneStep())
	{
		;
	}

	if (pDis != NULL)
	{
		// Copy distance data
		float distance;
		int numVer = m_pTriMesh->getVerNum();
		for (int verIdx = 0; verIdx < numVer; ++verIdx)
		{
			GW::GW_GeodesicVertex* vertex =
				(GW::GW_GeodesicVertex*)(m_pGeoMesh->GetVertex((GW::GW_U32)verIdx));

			if (vertex->GetState() > 1)
			{
				// This point is in the traversal list
				distance = vertex->GetDistance();

				pDis[verIdx] = distance;
			}
			else
			{
				pDis[verIdx] = -1.0f;
			}
		}
	}
}

void CGeodesicMesh::computePath(int startIdx, int endIdx)
{
	resetGeoMesh();
	addSeed(endIdx);

	//refinePath(startIdx);
}

void CGeodesicMesh::refinePath(int startIdx)
{
	m_zeroOrderPathIdxVec.clear();
	m_firstOrderPathIdxVec.clear();
	m_pathPointVec.clear();
	m_pathLength = 0.0f;

	GW::GW_GeodesicVertex* begin =
		(GW::GW_GeodesicVertex*)(m_pGeoMesh->GetVertex((GW::GW_U32)startIdx));
	if (!begin)
	{
		cout << "ERROR: BeginPointId was not found to lie on the mesh." << endl;
		return;
	}

	GW::GW_GeodesicPath track;
	track.ComputePath(*begin, FLT_MAX);

	GW::T_GeodesicPointList ptList = track.GetPointList();
	GW::GW_GeodesicPoint* pt;
	float parametricPos;
	GW::GW_GeodesicVertex *endVert1, *endVert2;
	GW::GW_Vector3D endPt1, endPt2;
	double pathPt[3], lastPathPt[3];
	int endPtId1, endPtId2, lastInsertedPtId = -1;

	const size_t nPts = ptList.size();
	m_pathPointVec = vector<vec3>(nPts);
	m_zeroOrderPathIdxVec = vector<int>(nPts);
	if (m_interOrder == 1)
	{
		m_firstOrderPathIdxVec = vector<int>(nPts * 2);
	}
	
	int idx = 0, idx0 = 0;
	for (GW::CIT_GeodesicPointList cit = ptList.begin(), citEnd = ptList.end();
		cit != citEnd; ++cit, ++idx, lastPathPt[0] = pathPt[0],
		lastPathPt[1] = pathPt[1], lastPathPt[2] = pathPt[2])
	{
		pt = *cit;

		// The parametric position of the vertex on the edge
		parametricPos = pt->GetCoord();

		// Get the end points of the edge on which the path lies.
		endVert1 = pt->GetVertex1();
		endVert2 = pt->GetVertex2();
		endPt1 = endVert1->GetPosition();
		endPt2 = endVert2->GetPosition();
		endPtId1 = endVert1->GetID();
		endPtId2 = endVert2->GetID();

		// Store the edge point ids. The ZerothOrderPointIds contain the closest
		// one. The FirstOrderPointIds contains the other one.
		if (parametricPos > 0.5)
		{
			if (lastInsertedPtId != endPtId1)
			{
				// avoid repeats
				lastInsertedPtId = endPtId1;

				m_zeroOrderPathIdxVec[idx0] = endPtId1;
				pathPt[0] = endPt1[0];
				pathPt[1] = endPt1[1];
				pathPt[2] = endPt1[2];

				if (m_interOrder == 0)
				{
					m_pathPointVec[idx0] = vec3(pathPt[0], pathPt[1], pathPt[2]);
				}

				++idx0;
			}

			if (m_interOrder == 1)
			{
				m_firstOrderPathIdxVec[2 * idx] = endPtId1;
				m_firstOrderPathIdxVec[2 * idx + 1] = endPtId2;
			}
		}
		else
		{
			if (lastInsertedPtId != endPtId2)
			{
				// avoid repeats
				lastInsertedPtId = endPtId2;

				m_zeroOrderPathIdxVec[idx0] = endPtId2;
				pathPt[0] = endPt2[0];
				pathPt[1] = endPt2[1];
				pathPt[2] = endPt2[2];

				if (m_interOrder == 0)
				{
					m_pathPointVec[idx0] = vec3(pathPt[0], pathPt[1], pathPt[2]);
				}

				++idx0;
			}

			if (m_interOrder == 1)
			{
				m_firstOrderPathIdxVec[2 * idx] = endPtId2;
				m_firstOrderPathIdxVec[2 * idx + 1] = endPtId1;
			}
		}

		if (m_interOrder == 1)
		{
			// Linearly interpolate the edge vertices based on the parametric
			// position
			pathPt[0] = parametricPos * endPt1[0] + (1 - parametricPos) * endPt2[0];
			pathPt[1] = parametricPos * endPt1[1] + (1 - parametricPos) * endPt2[1];
			pathPt[2] = parametricPos * endPt1[2] + (1 - parametricPos) * endPt2[2];

			m_pathPointVec[idx] = vec3(pathPt[0], pathPt[1], pathPt[2]);
		}

		// The curve length
		if (idx)
		{
			m_pathLength += sqrt(
				(lastPathPt[0] - pathPt[0]) * (lastPathPt[0] - pathPt[0])
				+ (lastPathPt[1] - pathPt[1]) * (lastPathPt[1] - pathPt[1])
				+ (lastPathPt[2] - pathPt[2]) * (lastPathPt[2] - pathPt[2])
				);
		}
	} // end loop over vertices in the gradient trace

	m_zeroOrderPathIdxVec.resize(idx0);
	if (m_interOrder == 0)
	{
		m_pathPointVec.resize(idx0);
	}
}


void CGeodesicMesh::getVertexPos(int idx, vec3* buff) {
	vec3* pVertices;
	pVertices = m_pTriMesh->getVertices();
	*buff = pVertices[idx];
}