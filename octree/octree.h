#pragma once

/**,
 * Auth :   liubo
 * Date :   2021/06/11 15:28
 * Comment: 八叉树: 增加，删除，查找  
 */
 

#include <vector>
#include <memory>
#include <algorithm>
#include <assert.h>

#if _DEBUG
#include <string>
#endif

namespace octree
{
	const int MaxElementCountInNode = 8;
	const size_t OcCount = 8;

	constexpr void Assert(bool b)
	{
		assert(b);
	}

	struct FOctVector
	{
		float X = 0;
		float Y = 0;
		float Z = 0;
		FOctVector() = default;
		FOctVector(float InX, float InY, float InZ)
			: X(InX), Y(InY), Z(InZ)
		{
		}
		static FOctVector Zero() { return FOctVector(0, 0, 0); }
		static FOctVector One() { return FOctVector(1, 1, 1); }
	};
	//const FVector FVector::Zero() = FVector(0, 0, 0);
	//const FVector FVector::One() = FVector(1, 1, 1);

	inline FOctVector operator+(const FOctVector& A, const FOctVector& B)
	{
		return FOctVector(A.X + B.X, A.Y + B.Y, A.Z + B.Z);
	}
	inline FOctVector operator-(const FOctVector& A, const FOctVector& B)
	{
		return FOctVector(A.X - B.X, A.Y - B.Y, A.Z - B.Z);
	}
	inline FOctVector operator*(const FOctVector& A, const FOctVector& B)
	{
		return FOctVector(A.X * B.X, A.Y * B.Y, A.Z * B.Z);
	}
	inline FOctVector operator*(const FOctVector& A, float B)
	{
		return A * FOctVector(B, B, B);
	}
	template<class T>
	inline T Min(const T& A, const T& B)
	{
		return A < B ? A : B;
	}

	template<class T>
	inline T Max(const T& A, const T& B)
	{
		return A > B ? A : B;
	}
	inline FOctVector Min(const FOctVector& A, const FOctVector& B)
	{
		return FOctVector(Min(A.X, B.X), Min(A.Y, B.Y), Min(A.Z, B.Z));
	}
	inline FOctVector Max(const FOctVector& A, const FOctVector& B)
	{
		return FOctVector(Max(A.X, B.X), Max(A.Y, B.Y), Max(A.Z, B.Z));
	}

	struct FOctBound
	{
		FOctVector Center;
		FOctVector Extent;

		FOctBound() = default;
		FOctBound(const FOctVector& InCenter, const FOctVector& InExtent)
			: Center(InCenter), Extent(InExtent)
		{
		}

		static FOctBound One() 
		{
			return FOctBound{ FOctVector::Zero(), FOctVector::One() * 0.5f };
		}

		bool Intersect(const FOctBound& Other) const
		{
			auto Pt0 = GetMin();
			auto Pt1 = GetMax();

			auto Pt2 = Other.GetMin();
			auto Pt3 = Other.GetMax();

			if (Pt2.X > Pt1.X || Pt2.Y > Pt1.Y || Pt2.Z > Pt1.Z)
			{
				return false;
			}

			if (Pt0.X > Pt3.X || Pt0.Y > Pt3.Y || Pt0.Z > Pt3.Z)
			{
				return false;
			}

			return true;
		}
		bool Contains(const FOctVector& Pt) const
		{
			auto Pt0 = GetMin();
			auto Pt1 = GetMax();
			return (Pt.X >= Pt0.X && Pt.X <= Pt1.X)
				&& (Pt.Y >= Pt0.Y && Pt.Y <= Pt1.Y)
				&& (Pt.Z >= Pt0.Z && Pt.Z <= Pt1.Z);
		}
		FOctVector GetMin() const
		{
			return Center - Extent;
		}
		FOctVector GetMax() const
		{
			return Center + Extent;
		}
		FOctBound Scale(FOctVector ScaleSize) const
		{
			return FOctBound{ Center, Extent * ScaleSize };
		}
		FOctBound Offset(FOctVector OffsetPos) const
		{
			return FOctBound{ Center + OffsetPos, Extent };
		}
	};
	inline FOctBound operator+(const FOctBound& A, const FOctBound& B)
	{
		auto Pt0 = Min(A.GetMin(), B.GetMin());
		auto Pt1 = Max(A.GetMax(), B.GetMax());

		auto Center = (Pt0 + Pt1) * 0.5f;
		auto Extent = (Pt1 - Pt0) * 0.5f;

		return FOctBound{ Center, Extent };
	}

	// 包含关系
	static bool Encapsulates(const FOctBound& outerBounds, const FOctBound& innerBounds)
	{
		return outerBounds.Contains(innerBounds.GetMin()) && outerBounds.Contains(innerBounds.GetMax());
	}

	//using T = int;
	
	template<class T>
	class FBoundsOctreeNode
	{
	public:
		struct FElement
		{
			T Obj;
			FOctBound Bound;

		private:
			FElement() = default;
			FElement(const T& InObj, const FOctBound& InBound) : Obj(InObj), Bound(InBound)
			{}
			friend class FBoundsOctreeNode<T>;
			//friend class std::_Ref_count_obj<FElement>;
		};

		//typedef std::shared_ptr<FElement> FElementPtr;
		using FElementPtr = std::shared_ptr<FElement>;
		typedef std::shared_ptr<FBoundsOctreeNode> FBoundsOctreeNodePtr;

		static FElementPtr MakeElement(const T& InObj, const FOctBound& InBound)
		{
			//return std::make_shared<FBoundsOctreeNode::FElement>(InObj, InBound);
			auto Ptr = new FElement(InObj, InBound);
			return std::shared_ptr< FElement>(Ptr);
		}
		static FBoundsOctreeNodePtr MakeNode(float InDiameter, float InMinSize, float InLooseness, const FOctVector& InCenter)
		{
			//return std::make_shared<FBoundsOctreeNode>(InDiameter, InMinSize, InLooseness, InCenter);
			return std::shared_ptr<FBoundsOctreeNode>(new FBoundsOctreeNode(InDiameter, InMinSize, InLooseness, InCenter));
		}

	private:
		FBoundsOctreeNode(float InDiameter, float InMinSize, float InLooseness, const FOctVector& InCenter)
		{
			Diameter = InDiameter;
			MinSize = InMinSize;
			Looseness = InLooseness;
			AdjustLength = Diameter * Looseness;

			LooseBound.Center = InCenter;
			LooseBound.Extent = FOctVector{ AdjustLength * 0.5f, AdjustLength * 0.5f, AdjustLength * 0.5f };
		}

	public:
		bool Add(const T& One, FOctBound InBound)
		{
			if (!Encapsulates(LooseBound, InBound))
			{
				return false;
			}
			
			SubAdd(One, InBound);
			return true;
		}
		bool Remove(const T& One, FOctBound InBound)
		{
			if (!Encapsulates(LooseBound, InBound))
			{
				return false;
			}

			SubRemove(One, InBound);
		}
		bool HasElement(const T& One)
		{
			for (auto It : Elements)
			{
				if (It->Obj == One)
				{
					return true;
				}
			}
			return false;
		}
		bool IsLeaf() const
		{
			return NodeList.size() == 0;
		}

#if _DEBUG
		int GetMaxDepth(int Depth) const
		{
			int MinDepth = Depth;
			for (auto It : NodeList)
			{
				auto V = It->GetMaxDepth(Depth + 1);
				if (V > MinDepth)
				{
					MinDepth = V;
				}
			}
			return MinDepth;
		}
		size_t GetAllNodeCount() const
		{
			size_t Cnt = 0;
			for (auto It : NodeList)
			{
				Cnt += It->GetAllNodeCount();
			}
			Cnt += NodeList.size();
			return Cnt;
		}
		void DumpNodeInfo(int Depth) const
		{
			if (Elements.size() + NodeList.size() > 0)
			{
				auto Pad = std::string(Depth * 4, ' ');
				std::cout << Pad << Elements.size() << std::endl;
			}

			for (auto It : NodeList)
			{
				It->DumpNodeInfo(Depth + 1);
			}
		}
#endif

	protected:
		void SubAdd(const T& One, const FOctBound& InBound)
		{
			SubAdd(MakeElement(One, InBound));
		}
		void SubAdd(const FElementPtr& Ptr)
		{
			if (NodeList.size() == 0)
			{
				if (Elements.size() < MaxElementCountInNode || TooSmall())
				{
					Elements.push_back(Ptr);
				}
				else
				{
					SplitNode();

					auto Old = Elements;
					Elements.clear();

					for (auto It : Old)
					{
						SubAdd(It->Obj, It->Bound);
					}

					SubAdd(Ptr);
				}
			}
			else
			{
				int NodeId = BestFitChild(Ptr->Bound.Center);
				if (Encapsulates(NodeList[NodeId]->LooseBound, Ptr->Bound))
				{
					NodeList[NodeId]->SubAdd(Ptr);
				}
				else
				{
					Elements.push_back(Ptr);
				}
			}
		}
		bool SubRemove(const T& One, const FOctBound& InBound)
		{
			bool bRemoved = false;

			{
				auto It = std::find_if(Elements.begin(), Elements.end(), [&One](const FElementPtr& A)
					{
						return A->Obj == One;
					});
				if (It != Elements.end())
				{
					Elements.erase(It);
					bRemoved = true;
				}
			}

			if (!bRemoved && NodeList.size() > 0)
			{
				int NodeId = BestFitChild(InBound.Center);
				bRemoved = NodeList[NodeId]->SubRemove(One, InBound);
			}

			if (bRemoved && NodeList.size() > 0)
			{
				if (ShouldMerge())
				{
					Merge();
				}
			}
		}

		// 是否与自己相交
		bool IsColliding(const FOctBound& InBound) const
		{
			if (!LooseBound.Intersect(InBound))
			{
				return false;
			}

			for (auto It : Elements)
			{
				if (It->Bound.Intersect(InBound))
				{
					return true;
				}
			}

			for (auto It : NodeList)
			{
				if (It->IsColliding(InBound))
				{
					return true;
				}
			}

			return false;
		}

		std::vector<FElementPtr> GetColliding(const FOctBound& InBound)
		{
			std::vector<FElementPtr> OutElements;
			GetColliding(InBound, OutElements);
			return OutElements;
		}
		void GetColliding(const FOctBound& InBound, std::vector<FElementPtr>& OutElements)
		{
			if (!LooseBound.Intersect(InBound))
			{
				return;
			}

			for (auto It : Elements)
			{
				if (It->Bound.Intersect(InBound))
				{
					OutElements.push_back(It);
				}
			}

			for (auto It : NodeList)
			{
				It->GetColliding(InBound, OutElements);
			}
		}

		static FBoundsOctreeNodePtr FindElement(FBoundsOctreeNodePtr This, const T& Obj, const FOctBound& InBound)
		{
			if (!This->LooseBound.Intersect(InBound))
			{
				return nullptr;
			}

			for (auto It : This->Elements)
			{
				if (It->Obj == Obj)
				{
					return This;
				}
			}

			for (auto It : This->NodeList)
			{
				auto Ret = FindElement(It, Obj, InBound);
				if (Ret != nullptr)
				{
					return Ret;
				}
			}

			return nullptr;
		}

		// 合并
		void Merge()
		{
			for (auto It : NodeList)
			{
				// 确保子节点没有子节点
				Assert(It->NodeList.size() == 0);

				// 将子节点的内容合并到自己身上
				for (auto Jt : It->Elements)
				{
					Elements.push_back(Jt);
				}
			}

			// 删掉子节点
			NodeList.clear();
		}

		// 是否合并
		bool ShouldMerge() const
		{
			int Cnt = 0;
			for (auto It : NodeList)
			{
				// 如果子节点还有子节点，那么就不合并了
				if (It->NodeList.size() > 0)
				{
					return false;
				}
				Cnt += It->Elements.size();
			}
			Cnt += Elements.size();

			return Cnt < MaxElementCountInNode;
		}



		size_t BestFitChild(const FOctVector& ObjBoundsCenter) const
		{
			return (ObjBoundsCenter.X <= LooseBound.Center.X ? 0 : 1) 
				+ (ObjBoundsCenter.Y >= LooseBound.Center.Y ? 0 : 4)
				+ (ObjBoundsCenter.Z <= LooseBound.Center.Z ? 0 : 2);
		}
		FOctVector NodeIdxToDir(int NodeId)
		{
			FOctVector Ret;
			Ret.X = NodeId % 2 == 0 ? -1 : 1;
			Ret.Y = NodeId > 3 ? -1 : 1;
			Ret.Z = (NodeId < 2 || (NodeId > 3 && NodeId < 6)) ? -1 : 1;
			return Ret;
		}
		void SplitNode()
		{
			Assert(NodeList.size() == 0);

			float NewLength = Diameter / 2;
			float NewCenter = Diameter / 4;

			NodeList.push_back(MakeNode(NewLength, MinSize, Looseness,
				LooseBound.Center + FOctVector(-NewCenter, NewCenter, -NewCenter)));
			NodeList.push_back(MakeNode(NewLength, MinSize, Looseness,
				LooseBound.Center + FOctVector(NewCenter, NewCenter, -NewCenter)));
			NodeList.push_back(MakeNode(NewLength, MinSize, Looseness,
				LooseBound.Center + FOctVector(-NewCenter, NewCenter, NewCenter)));
			NodeList.push_back(MakeNode(NewLength, MinSize, Looseness,
				LooseBound.Center + FOctVector(NewCenter, NewCenter, NewCenter)));

			NodeList.push_back(MakeNode(NewLength, MinSize, Looseness,
				LooseBound.Center + FOctVector(-NewCenter, -NewCenter, -NewCenter)));
			NodeList.push_back(MakeNode(NewLength, MinSize, Looseness,
				LooseBound.Center + FOctVector(NewCenter, -NewCenter, -NewCenter)));
			NodeList.push_back(MakeNode(NewLength, MinSize, Looseness,
				LooseBound.Center + FOctVector(-NewCenter, -NewCenter, NewCenter)));
			NodeList.push_back(MakeNode(NewLength, MinSize, Looseness,
				LooseBound.Center + FOctVector(NewCenter, -NewCenter, NewCenter)));

			Assert(NodeList.size() == OcCount);
		}
		bool TooSmall() const 
		{ 
			return Diameter < MinSize;
		}
		bool HasAnyObjects() const
		{
			if (Elements.size() > 0)
			{
				return true;
			}
			if (NodeList.size() > 0)
			{
				return true;
			}
			return false;
		}

	protected:
		std::vector<FElementPtr> Elements;
		std::vector<FBoundsOctreeNodePtr> NodeList;
		float Diameter = 0;
		float MinSize = 0;
		float Looseness = 1;
		float AdjustLength = 0;
		FOctBound LooseBound;

		template<class T> friend class FBoundsOctree;
	};

	template<typename T>
	class FBoundsOctree
	{
	public:
		//typedef FBoundsOctreeNode<T> InsideNodeType;
		using InsideNodeType = typename FBoundsOctreeNode<T>;
		//using InsideNodeTypeElementPtr = typename FBoundsOctreeNode<T>::FElementPtr;
		typedef typename InsideNodeType::FElementPtr InsideNodeType_ElementPtr;
		using InsideNodeType_NodePtr = typename InsideNodeType::FBoundsOctreeNodePtr;
	public:
		FBoundsOctree(float initialDiameter, const FOctVector& WorldPos, float MinNodeSize, float InLooseness)
			: InitialDiameter(initialDiameter), MinSize(MinNodeSize), Looseness(InLooseness), MyCenter(WorldPos)
		{
			RootNode = InsideNodeType::MakeNode(InitialDiameter, MinSize, Looseness, MyCenter);
		}

		void Add(const T& One, FOctBound InBound)
		{
			int TryCount = 0;
			while (!RootNode->Add(One, InBound))
			{
				Grow(InBound.Center - MyCenter);
				TryCount++;
				if (TryCount >= 20)
				{
					Assert(false);
					return;
				}
			}
			Count++;
		}
		bool Remove(const T& One, FOctBound InBound)
		{
			bool b = RootNode->Remove(One, InBound);

			if (b)
			{
				Count--;
				Shrink();
			}
		}
		
		bool IsColliding(const FOctBound& InBound) const
		{
			return RootNode->IsColliding(InBound);
		}
		std::vector<InsideNodeType_ElementPtr> GetColliding(const FOctBound& InBound)
		{
			return RootNode->GetColliding(InBound);
		}

		InsideNodeType_NodePtr FindElement(const T& Obj, const FOctBound& InBound)
		{
			auto Ret = InsideNodeType::FindElement(RootNode, Obj, InBound);
			return Ret;
		}

		int GetCount() const { return Count; }
		float GetTopSize() const { return RootNode->Diameter; }

#if _DEBUG
		int GetMaxDepth() const 
		{
			return RootNode->GetMaxDepth(0);
		}
		int GetAllNodeCount() const
		{			
			return RootNode->GetAllNodeCount();
		}
		void DumpNodeInfo() const
		{
			return RootNode->DumpNodeInfo(0);
		}
#endif

	protected:

		void Shrink()
		{
		
		}

		// 扩大自己的范围
		void Grow(const FOctVector& Dir)
		{
			int X = Dir.X >= 0 ? 1 : -1;
			int Y = Dir.Y >= 0 ? 1 : -1;
			int Z = Dir.Z >= 0 ? 1 : -1;

			auto OldRoot = RootNode;
			auto NewLength = OldRoot->Diameter * 2;
			auto HalfLength = OldRoot->Diameter / 2;

			// 把旧Root想象成一个立方体，那么新的Root的位置一定是旧Root的6个顶点中的一个
			auto CenterOffset = FOctVector(X * HalfLength, Y * HalfLength, Z * HalfLength);
			auto NewCenter = MyCenter + CenterOffset;
			
			RootNode = InsideNodeType::MakeNode(NewLength, MinSize, Looseness, NewCenter);
			if (OldRoot->HasAnyObjects())
			{
				// 将旧的整合进来，构建8个子节点，其中一个是OldRoot
				std::vector<InsideNodeType_NodePtr> NodeList(8, nullptr);
				Assert(NodeList.size() == OcCount);
				auto NodeId = RootNode->BestFitChild(OldRoot->LooseBound.Center);
				{
					for (size_t i = 0; i < NodeList.size(); i++)
					{
						if (i == NodeId)
						{
							NodeList[i] = OldRoot;
						}
						else
						{
							FOctVector CenterOffset2 = RootNode->NodeIdxToDir(i);

							CenterOffset2.X = HalfLength * CenterOffset2.X;
							CenterOffset2.Y = HalfLength * CenterOffset2.Y;
							CenterOffset2.Z = HalfLength * CenterOffset2.Z;

							NodeList[i] = InsideNodeType::MakeNode(OldRoot->Diameter,
								OldRoot->MinSize, OldRoot->Looseness, NewCenter + CenterOffset2);
						}
					}
				}
				RootNode->NodeList = NodeList;
			}
		}

	protected:
		InsideNodeType_NodePtr RootNode;
		float InitialDiameter = 0;
		float MinSize = 0;
		float Looseness = 1;
		int Count = 0;
		FOctVector MyCenter;
	};




}
