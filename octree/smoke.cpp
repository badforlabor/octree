/**,
 * Auth :   liubo
 * Date :   2021/06/11 15:28
 * Comment: 测试代码  
 */
 
#include <iostream>
#include <assert.h>
#include <stdlib.h>
#include <windows.h>
#include <ppl.h>
#include <chrono> 

#include "octree.h"


float Rand(float A, float B)
{
	return (1.0f * rand() / RAND_MAX) * (B - A);
}

class SmokeTestTag
{
public:
	SmokeTestTag(const char* Tag) : TagStr(Tag)
	{
		std::cout << "-------------------" << " " << TagStr.c_str();
		std::cout << " Start" << " " << "-------------------" << std::endl;
	}
	~SmokeTestTag()
	{
		std::cout << "-------------------" << " " << TagStr.c_str();
		std::cout << " End" << " " << "-------------------" << std::endl;
	}

private:
	std::string TagStr;
};

void TestBound()
{
	SmokeTestTag Tag("TestBound");

	using namespace octree;

	
	const FOctBound One = FOctBound::One();
	const FOctBound SameOne = One;
	const FOctBound HalfOne = One.Scale(FOctVector::One() * 0.5f);
	const FOctBound BigOne = One.Scale(FOctVector::One() * 2);

	
	// 9宫格判定
	std::vector<FOctVector> OffsetList;
	for (int i = -1; i <= 1; i++)
	{
		for (int j = -1; j <= 1; j++)
		{
			for (int k = -1; k <= 1; k++)
			{
				OffsetList.push_back(FOctVector(i, j, k));
			}
		}
	}
	for (size_t i = 0; i < OffsetList.size(); i++)
	{
		FOctBound B1 = One;
		FOctBound B2 = SameOne.Offset(OffsetList[i]);

		assert(B1.Intersect(B2));
		assert(B2.Intersect(B1));
	}
	for (size_t i = 0; i < OffsetList.size(); i++)
	{
		FOctBound B1 = One;
		FOctBound B2 = HalfOne.Offset(OffsetList[i]);

		if (OffsetList[i].X == 0 && OffsetList[i].Y == 0 && OffsetList[i].Z == 0)
		{
			assert(B1.Intersect(B2));
			assert(B2.Intersect(B1));
		}
		else
		{
			assert(!B1.Intersect(B2));
			assert(!B2.Intersect(B1));
		}
	}
	for (size_t i = 0; i < OffsetList.size(); i++)
	{
		FOctBound B1 = One;
		FOctBound B2 = BigOne.Offset(OffsetList[i]);

		assert(B1.Intersect(B2));
		assert(B2.Intersect(B1));
	}


	for (size_t i = 0; i < OffsetList.size(); i++)
	{
		FOctBound B1 = One;
		FOctBound B2 = SameOne.Offset(OffsetList[i] * 1.1f);

		if (OffsetList[i].X == 0 && OffsetList[i].Y == 0 && OffsetList[i].Z == 0)
		{
			assert(B1.Intersect(B2));
			assert(B2.Intersect(B1));
		}
		else
		{
			assert(!B1.Intersect(B2));
			assert(!B2.Intersect(B1));
		}
	}
	for (size_t i = 0; i < OffsetList.size(); i++)
	{
		FOctBound B1 = One;
		FOctBound B2 = BigOne.Offset(OffsetList[i] * 2.1);

		if (OffsetList[i].X == 0 && OffsetList[i].Y == 0 && OffsetList[i].Z == 0)
		{
			assert(B1.Intersect(B2));
			assert(B2.Intersect(B1));
		}
		else
		{
			assert(!B1.Intersect(B2));
			assert(!B2.Intersect(B1));
		}
	}

	


	{
		FOctBound B1{ FOctVector::Zero(), One.Extent };
		FOctBound B2{ FOctVector::One() * 0.5f, One.Extent };
		assert(B1.Intersect(B2));
		assert(B2.Intersect(B1));
	}

	{
		FOctBound B1{ FOctVector::Zero(), One.Extent };
		FOctBound B2{ FOctVector::One(), One.Extent };
		assert(B1.Intersect(B2));
		assert(B2.Intersect(B1));
	}

	{
		FOctBound B1{ FOctVector::Zero(), One.Extent };
		FOctBound B2{ FOctVector::One() * 1.1f, One.Extent };
		assert(!B1.Intersect(B2));
		assert(!B2.Intersect(B1));
	}

	{
		FOctBound B1{ FOctVector::Zero(), One.Extent };
		FOctBound B2{ FOctVector::One() * 1.5f, One.Extent };
		assert(!B1.Intersect(B2));
		assert(!B2.Intersect(B1));
	}


	// TestContain

	for (size_t i = 0; i < OffsetList.size(); i++)
	{
		FOctBound B1 = One;
		FOctVector Pt = FOctVector::Zero() + OffsetList[i] * 0.5f;

		assert(B1.Contains(Pt));
	}

	for (size_t i = 0; i < OffsetList.size(); i++)
	{
		FOctBound B1 = One;
		FOctVector Pt = FOctVector::Zero() + OffsetList[i] * 0.5f * 1.1f;

		if (OffsetList[i].X == 0 && OffsetList[i].Y == 0 && OffsetList[i].Z == 0)
		{
			assert(B1.Contains(Pt));
		}
		else
		{
			assert(!B1.Contains(Pt));
		}
	}
}

namespace octree_smoke
{
	struct ClassA
	{
		int V1;
		float V2;
	};
	bool operator==(const ClassA& a, const ClassA& b)
	{
		return a.V1 == b.V1 && a.V2 == b.V2;
	}
}

void TestTree()
{
	const float UnitMeter = 100;

	using namespace octree;
	using namespace octree_smoke;


	using FBoundsOctreeA = octree::FBoundsOctree<ClassA>;
	FBoundsOctreeA Tree(UnitMeter * 10, FOctVector::Zero(), UnitMeter * 0.1, 1.5f);

	float MinSize = UnitMeter * 0.01f;
	float MaxSize = UnitMeter * 10;
	float MaxDiameter = 100 * UnitMeter;

	float MinRadius = MinSize * 0.5f;
	float MaxRadius = MaxSize * 0.5f;
	float HalfMaxDiameter = MaxDiameter * 0.5f;

	srand(1);

	std::vector< FBoundsOctreeA::InsideNodeType_ElementPtr> ActorList;
	for (int i = 0; i < 10000; i++)
	{
		FOctVector Pos(Rand(-HalfMaxDiameter, HalfMaxDiameter), Rand(-HalfMaxDiameter, HalfMaxDiameter), Rand(-HalfMaxDiameter, HalfMaxDiameter));
		FOctVector Extent(Rand(MinRadius, MaxRadius), Rand(MinRadius, MaxRadius), Rand(MinRadius, MaxRadius));

		auto One = FBoundsOctreeA::InsideNodeType::MakeElement(ClassA{i, i * 3.14f}, FOctBound{ Pos, Extent });
		ActorList.push_back(One);
	}

	for (auto It : ActorList)
	{
		Tree.Add(It->Obj, It->Bound);
	}
	assert(Tree.GetCount() == ActorList.size());

	/**************************************************************************/
	/* 测试正确性                                                             */
	/**************************************************************************/

	// 任意一个点，都能找到
	{
		auto StartTime = std::chrono::system_clock::now();
		for (auto It : ActorList)
		{
			auto List = Tree.GetColliding(It->Bound);
			auto FindIt = std::find_if(List.begin(), List.end(), [It](auto Ptr)
				{
					return It->Obj == Ptr->Obj;
				});
			assert(FindIt != List.end());
		}

		auto End = std::chrono::system_clock::now();
		auto Ms = std::chrono::duration_cast<std::chrono::microseconds>(End - StartTime);
		auto CostTime = Ms.count() / ActorList.size();
		std::cout << "平均查找时间:" << (CostTime / 1000.0f) << "ms" << std::endl;
	}


}