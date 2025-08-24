package main

import (
	"log"
	"net/http"

	"github.com/o0olele/detour-go/debugger"
)

// CORS middleware
func corsMiddleware(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Access-Control-Allow-Origin", "*")
		w.Header().Set("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
		w.Header().Set("Access-Control-Allow-Headers", "Content-Type, Authorization")

		if r.Method == "OPTIONS" {
			w.WriteHeader(http.StatusOK)
			return
		}

		next.ServeHTTP(w, r)
	})
}

func main() {
	// Create the debugger server
	server := debugger.NewServer()

	// Create main mux
	mux := http.NewServeMux()

	// Serve static files
	mux.Handle("/public/", http.StripPrefix("/public/", http.FileServer(http.Dir("./public"))))

	// Mount the debugger server routes
	mux.Handle("/", server)

	// Apply CORS middleware
	handler := corsMiddleware(mux)

	log.Println("Server starting on :9001")
	log.Fatal(http.ListenAndServe(":9001", handler))
}
