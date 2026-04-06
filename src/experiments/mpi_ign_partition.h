#ifndef MPI_IGN_PARTITION_H
#define MPI_IGN_PARTITION_H

#include <cstdlib>     // for setenv, getenv
#include <string>
#include <stdexcept>
#include <sstream>

/// @brief Sets the IGN_PARTITION environment variable based on MPI rank.
/// @param rank The MPI rank of the current process.
/// @return The partition name that was set.
inline std::string initIgnPartition(int rank)
{
    const char* existing = std::getenv("IGN_PARTITION");
    if (existing && *existing) {
        std::string partition(existing);
        std::cout << "using existing IGN_PARTITION " << partition << std::endl;
        return partition;
    }

    std::string partition = "ign_partition_" + std::to_string(rank);
    std::cout << "set ign_partition " << partition << std::endl;

    if (setenv("IGN_PARTITION", partition.c_str(), 1) != 0)
    {
        throw std::runtime_error("Failed to set IGN_PARTITION environment variable.");
    }
    return partition;
}



inline std::string getIgnPartition()
{
    const char* val = std::getenv("IGN_PARTITION");
    return (val != nullptr) ? std::string(val) : std::string();
}

#endif
