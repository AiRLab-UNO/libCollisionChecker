//
// Created by Stefan Annell on 2024-01-14.
//
#pragma once

#include "OctreeUtil.h"

template <IsDataWrapper TDataWrapper>
struct AllQuery {
    bool IsInside([[maybe_unused]] const TDataWrapper& Vector) const {
        return true;
    }
    bool Covers([[maybe_unused]] const Boundary<typename TDataWrapper::VectorType>& Boundary) const {
        return true;
    }
};

template <IsDataWrapper TDataWrapper>
struct SphereQuery {
    const typename TDataWrapper::VectorType Midpoint = {0, 0, 0};
    const float Radius = 0.0f;

    bool IsInside(const TDataWrapper& Data) const {
        return DistanceSquared(Midpoint, Data.Vector) <= Radius * Radius;
    }

    bool Covers(const Boundary<typename TDataWrapper::VectorType>& Boundary) const {
        return CheckOverlapp(Boundary, Midpoint, Radius);
    }
};

template <IsDataWrapper TDataWrapper>
struct CircleQuery {
    const typename TDataWrapper::VectorType Midpoint = {0, 0};
    const float Radius = 0.0f;

    bool IsInside(const TDataWrapper& Data) const {
        return DistanceSquared(Midpoint, Data.Vector) <= Radius * Radius;
    }

    bool Covers(const Boundary<typename TDataWrapper::VectorType>& Boundary) const {
        return CheckOverlapp(Boundary, Midpoint, Radius);
    }
};

template <IsDataWrapper TDataWrapper>
struct CubeQuery {
    const typename TDataWrapper::VectorType MinCorner = {0, 0, 0};
    const typename TDataWrapper::VectorType MaxCorner = {0, 0, 0};

    bool IsInside(const TDataWrapper& Data) const {
        const auto& p = Data.Vector;
        return (p.x >= MinCorner.x && p.x <= MaxCorner.x &&
                p.y >= MinCorner.y && p.y <= MaxCorner.y &&
                p.z >= MinCorner.z && p.z <= MaxCorner.z);
    }

    bool Covers(const Boundary<typename TDataWrapper::VectorType>& Boundary) const {
        // Returns true if the box defined by MinCorner/MaxCorner fully covers the given boundary
        return (MinCorner.x <= Boundary.Min.x && MaxCorner.x >= Boundary.Max.x &&
                MinCorner.y <= Boundary.Min.y && MaxCorner.y >= Boundary.Max.y &&
                MinCorner.z <= Boundary.Min.z && MaxCorner.z >= Boundary.Max.z);
    }
};

template <IsDataWrapper TDataWrapper>
struct CylinderQuery {
    const typename TDataWrapper::VectorType Point1 = {0, 0, 0};
    const typename TDataWrapper::VectorType Point2 = {0, 0, 0};
    const float Radius = 0.0f;

    bool IsInside(const TDataWrapper& Data) const {
        return DistancePointToLine(Data.Vector, Point1, Point2) <= Radius;
    }

    bool Covers(const Boundary<typename TDataWrapper::VectorType>& Boundary) const {
        return IsBoxInsideCylinder(Boundary, Point1, Point2, Radius);
    }
};

template <IsDataWrapper TDataWrapper>
struct PredQuery {
    std::function<bool(const TDataWrapper&)> Pred;

    bool IsInside(const TDataWrapper& Data) const {
        return Pred(Data);
    }

    bool Covers([[maybe_unused]] const Boundary<typename TDataWrapper::VectorType>& Boundary) const {
        return true;
    }
};

template <IsDataWrapper TDataWrapper, IsQuery<TDataWrapper> QueryLHS, IsQuery<TDataWrapper> QueryRHS>
struct AndQuery {
    QueryLHS Query1;
    QueryRHS Query2;

    bool IsInside(const TDataWrapper& Data) const {
        return Query1.IsInside(Data) && Query2.IsInside(Data);
    }

    bool Covers(const Boundary<typename TDataWrapper::VectorType>& Boundary) const {
        return Query1.Covers(Boundary) && Query2.Covers(Boundary);
    }
};

template <IsDataWrapper TDataWrapper, IsQuery<TDataWrapper> QueryLHS, IsQuery<TDataWrapper> QueryRHS>
struct OrQuery {
    QueryLHS Query1;
    QueryRHS Query2;

    bool IsInside(const TDataWrapper& Data) const {
        return Query1.IsInside(Data) || Query2.IsInside(Data);
    }

    bool Covers(const Boundary<typename TDataWrapper::VectorType>& Boundary) const {
        return Query1.Covers(Boundary) || Query2.Covers(Boundary);
    }
};

template <IsDataWrapper TDataWrapper, IsQuery<TDataWrapper> TQuery>
struct NotQuery {
    TQuery Query;

    bool IsInside(const TDataWrapper& Data) const {
        return !Query.IsInside(Data);
    }

    bool Covers([[maybe_unused]] const Boundary<typename TDataWrapper::VectorType>& Boundary) const {
        return true;
    }
};

