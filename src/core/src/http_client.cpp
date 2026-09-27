#include <basestation/core/http_client.h>

#ifdef BASESTATION_HAVE_CURL
#include <curl/curl.h>
#endif

#include <mutex>

namespace basestation::http {

#ifdef BASESTATION_HAVE_CURL

namespace {

struct Transfer {
    Response* resp;
    size_t max_bytes;
    const std::atomic<bool>* abort;
    bool too_big = false;
};

size_t on_body(char* data, size_t size, size_t n, void* user) {
    auto* t = static_cast<Transfer*>(user);
    const size_t len = size * n;
    if (t->resp->body.size() + len > t->max_bytes) {
        t->too_big = true;
        return 0;  // makes curl fail with CURLE_WRITE_ERROR
    }
    t->resp->body.insert(t->resp->body.end(), data, data + len);
    return len;
}

int on_progress(void* user, curl_off_t, curl_off_t, curl_off_t, curl_off_t) {
    const auto* t = static_cast<const Transfer*>(user);
    return t->abort && t->abort->load() ? 1 : 0;  // non-zero aborts
}

}  // namespace

bool available() { return true; }

void global_init() {
    static std::once_flag once;
    std::call_once(once, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
}

Response get(const std::string& url, const Options& opts, const std::atomic<bool>* abort) {
    Response r;
    CURL* curl = curl_easy_init();
    if (!curl) {
        r.error = "curl_easy_init failed";
        r.transport_error = true;
        return r;
    }
    Transfer t{&r, opts.max_bytes, abort};
    char errbuf[CURL_ERROR_SIZE] = {};
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_USERAGENT, opts.user_agent.c_str());
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_MAXREDIRS, 3L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, opts.connect_timeout_s);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, opts.timeout_s);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);  // required with threads
    curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "");  // whatever curl supports
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, on_body);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &t);
    curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, on_progress);
    curl_easy_setopt(curl, CURLOPT_XFERINFODATA, &t);
    curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errbuf);

    const CURLcode rc = curl_easy_perform(curl);
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &r.status);
    char* ct = nullptr;
    if (curl_easy_getinfo(curl, CURLINFO_CONTENT_TYPE, &ct) == CURLE_OK && ct) r.content_type = ct;
    curl_easy_cleanup(curl);

    if (rc != CURLE_OK) {
        if (t.too_big) {
            r.error = "response larger than " + std::to_string(opts.max_bytes) + " bytes";
        } else if (rc == CURLE_ABORTED_BY_CALLBACK) {
            r.error = "cancelled";
        } else {
            r.error = errbuf[0] ? errbuf : curl_easy_strerror(rc);
            r.transport_error = true;
        }
        r.body.clear();
        return r;
    }
    if (r.status < 200 || r.status >= 300) {
        r.error = "HTTP " + std::to_string(r.status);
        r.body.clear();
        return r;
    }
    r.ok = true;
    return r;
}

#else  // !BASESTATION_HAVE_CURL

bool available() { return false; }
void global_init() {}

Response get(const std::string&, const Options&, const std::atomic<bool>*) {
    Response r;
    r.error = "built without libcurl: downloads are disabled";
    return r;
}

#endif

}  // namespace basestation::http
