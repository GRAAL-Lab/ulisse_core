#ifndef DIRECTORY_MANAGER_HPP
#define DIRECTORY_MANAGER_HPP

#include <boost/filesystem.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace fs = boost::filesystem;

class DirectoryManager {
public:
    DirectoryManager(const std::string& base_dir)
        : baseDir_(base_dir)
    {
        // Create timestamped directory upon construction
        timestamp_ = getFullTimestamp(); // Full timestamp with nanoseconds
        dirPath_ = fs::path(baseDir_) / timestamp_;

        try {
            if (!fs::create_directories(dirPath_)) {
                std::cerr << "Failed to create directory (may already exist): " << dirPath_ << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Exception creating directory: " << e.what() << std::endl;
        }
    }

    // Generate a file path with optional prefix, suffix, and extension
    std::string generateFilePath(const std::string& prefix = "", const std::string& suffix = "", const std::string& extension = ".txt", bool addTimestamp = true) const
    {
        std::stringstream ss;
        if (!prefix.empty())
            ss << prefix << "_";
        if (addTimestamp)
            ss << getFullTimestamp() << "_"; // Full timestamp with nanoseconds
        if (!suffix.empty())
            ss << suffix;
        ss << extension;
        return (dirPath_ / ss.str()).string();
    }

    // Open a file for writing and return a pointer to the ofstream object
    std::ofstream* openFile(const std::string& prefix = "", const std::string& suffix = "", const std::string& extension = ".json", bool addTimestamp = true)
    {
        std::string filePath = generateFilePath(prefix, suffix, extension, addTimestamp);
        std::ofstream* outFile = new std::ofstream(filePath);

        if (!outFile->is_open()) {
            std::cerr << "Error: Could not open file " << filePath << " for writing." << std::endl;
            delete outFile;
            return nullptr;
        }

        return outFile;
    }

    // Save content to the file using the passed pointer
    bool saveToFile(std::ofstream* outFile, const std::string& content)
    {
        if (outFile) {
            (*outFile) << content;
            closeFile(outFile);
            return true;
        } else {
            std::cerr << "Error: Invalid file pointer." << std::endl;
            return false;
        }
    }

    // Access the directory path
    std::string getDirPath() const
    {
        return dirPath_.string();
    }

    // Close the file pointer if it was opened
    void closeFile(std::ofstream* outFile)
    {
        if (outFile) {
            outFile->close();
            delete outFile;
        }
    }

private:
    std::string baseDir_;
    std::string timestamp_;
    fs::path dirPath_;

    // Get full timestamp including nanoseconds (format: YYYYMMDD_HHMMSS_NNNNNNNNN)
    std::string getFullTimestamp() const
    {
        auto now = std::chrono::system_clock::now();
        auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
        auto duration = now_ns.time_since_epoch();

        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&now_c);

        std::stringstream ss;
        ss << std::put_time(&tm, "%Y%m%d_%H%M%S") << "_" << std::setw(9) << std::setfill('0') << nanoseconds.count();
        return ss.str();
    }
};
#endif