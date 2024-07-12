#include <iostream>
#include <curl/curl.h>
#include <string>

// Callback function for data received from the server
size_t WriteCallback(void *contents, size_t size, size_t nmemb, std::string *userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

// Function to make a GET request to the specified URL
void makeRequest(const std::string& jointName, const std::string& value = "") {
    CURL *curl;
    CURLcode res;
    std::string readBuffer;

    curl = curl_easy_init();
    if(curl) {
        std::string url = "http://192.168.93.194:80/" + jointName+value;
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

        // Perform the request, res will get the return code
        res = curl_easy_perform(curl);
        // Check for errors
        if(res != CURLE_OK)
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        else
            std::cout << "Response from server: " << readBuffer << std::endl;

        // Always cleanup
        curl_easy_cleanup(curl);
    }
}

int main() {
    std::string jointName = "moveServo"; // Replace with your joint name
    makeRequest(jointName, "?joint=servo2&value=90");
    makeRequest(jointName, "?joint=servo3&value=90");
    return 0;
}