/**,
 * Auth :   liubo
 * Date :   2021/06/11 15:28
 * Comment: 例子
 */
 

#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <ppl.h>
#include <chrono> 

#include "octree.h"

extern float Rand(float A, float B);

int main()
{
	extern void TestBound();
	TestBound();
	
	const float UnitMeter = 100;

	using namespace octree;
	using FBoundsOctreeInt = octree::FBoundsOctree<int>;
	FBoundsOctreeInt Tree(UnitMeter * 10, FOctVector::Zero(), UnitMeter * 0.1, 1.5f);
	
	{
		float MinSize = UnitMeter * 0.01f;
		float MaxSize = UnitMeter * 10;
		float MaxDiameter = 100 * UnitMeter;

		float MinRadius = MinSize * 0.5f;
		float MaxRadius = MaxSize * 0.5f;
		float HalfMaxDiameter = MaxDiameter * 0.5f;

		srand(1);

		std::vector< FBoundsOctreeInt::InsideNodeType_ElementPtr> ActorList;
		for (int i = 0; i < 10000; i++)
		{
			FOctVector Pos(Rand(-HalfMaxDiameter, HalfMaxDiameter), Rand(-HalfMaxDiameter, HalfMaxDiameter), Rand(-HalfMaxDiameter, HalfMaxDiameter));
			FOctVector Extent(Rand(MinRadius, MaxRadius), Rand(MinRadius, MaxRadius), Rand(MinRadius, MaxRadius));

			auto One = FBoundsOctreeInt::InsideNodeType::MakeElement(i, FOctBound{ Pos, Extent });
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

		{
			auto One = ActorList[10];
			auto Ele = Tree.FindElement(One->Obj, One->Bound);
			assert(Ele->HasElement(One->Obj));
#if _DEBUG
			Ele->DumpNodeInfo(0);
#endif
		}

		// 任意两个点，都能找到
		if (false)
		{
			int Cnt = 10;
			for (auto It : ActorList)
			{
				if (--Cnt < 0)
				{
					break;
				}

				concurrency::parallel_for_each(ActorList.begin(), ActorList.end(), [&Tree, &ActorList, &It](auto Two)
					{
						auto Bound = It->Bound + Two->Bound;
						auto List = Tree.GetColliding(Bound);
						auto FindIt = std::find_if(List.begin(), List.end(), [It, Two](auto Ptr)
							{
								return It->Obj == Ptr->Obj || It->Obj == Two->Obj;
							});
						assert(FindIt != List.end());
					});
			}
		}


		// dump
		std::cout << Tree.GetCount() << std::endl;
		std::cout << Tree.GetTopSize() << std::endl;
#if _DEBUG
		std::cout << Tree.GetMaxDepth() << std::endl;
		std::cout << Tree.GetAllNodeCount() << std::endl;
		//Tree.DumpNodeInfo();
#endif

	}


	extern void TestTree();
	TestTree();

    std::cout << "Hello World!\n";
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
