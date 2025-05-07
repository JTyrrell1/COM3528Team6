import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from pathlib import Path
from PIL import Image, ImageTk
import fitz  # PyMuPDF

from Team6_work.image_description import ImageDescriber
from Team6_work.vapi_therapist import Vapi_TheRapist


class ReminiscenceTherapyGUI(ttk.Frame):
    def __init__(self, root, patient_info, theme_path="Team6_work/Azure-ttk-theme-main/azure.tcl"):
        super().__init__(root, padding=20)
        self.root = root

        # Load Azure theme
        self.root.tk.call("source", theme_path)
        self.root.tk.call("set_theme", "light")

        # Configure window
        self.root.title("Reminiscence Therapy")
        try:
            self.root.state("zoomed")
        except tk.TclError:
            try:
                self.root.attributes("-zoomed", True)
            except tk.TclError:
                self.root.geometry(f"{self.root.winfo_screenwidth()}x{self.root.winfo_screenheight()}+0+0")

        self.root.minsize(800, 600)
        self.pack(fill="both", expand=True)

        # Patient data
        self.patient_name = str(patient_info.get("name", "Unknown"))
        self.patient_age = str(patient_info.get("age", "Unknown"))
        self.history_file_path = patient_info.get("history_path")
        self.history_text = None

        # Image data
        self.image_file_path = None
        self.image_type = tk.StringVar(value="personal")

        # Build UI
        self._build_image_panel()
        self._build_info_panel()

        self.columnconfigure(0, weight=3)
        self.columnconfigure(1, weight=2)
        self.rowconfigure(0, weight=1)

        # Load initial history preview if available
        if self.history_file_path:
            self._preview_pdf(self.history_file_path)
            self.history_text = self._extract_text_from_pdf(self.history_file_path)

        self.vapi_therapist = None

    def _build_image_panel(self):
        panel = ttk.LabelFrame(self, text="Reminiscence Image", padding=12)
        panel.grid(row=0, column=0, sticky="nsew", padx=(0, 20))

        self.image_display = ttk.Label(panel, text="No image uploaded", anchor="center", justify="center")
        self.image_display.pack(expand=True, fill="both")

        ttk.Button(panel, text="Upload Image", command=self._upload_image).pack(pady=8)

        desc_frame = ttk.LabelFrame(panel, text="Image Description", padding=8)
        desc_frame.pack(fill="x", pady=(10, 0))

        self.image_description_widget = tk.Text(desc_frame, height=5, wrap="word", borderwidth=0, highlightthickness=0)
        self.image_description_widget.pack(fill="both", expand=True)

    def _build_info_panel(self):
        panel = ttk.Frame(self)
        panel.grid(row=0, column=1, sticky="nsew")

        info_frame = ttk.LabelFrame(panel, text="Patient Info", padding=12)
        info_frame.pack(fill="x")
        ttk.Label(info_frame, text=f"Name: {self.patient_name}").pack(anchor="w")
        ttk.Label(info_frame, text=f"Age: {self.patient_age}").pack(anchor="w")

        self.history_label = ttk.Label(info_frame, text="History: none uploaded", foreground="grey")
        self.history_label.pack(anchor="w", pady=(5, 0))
        ttk.Button(info_frame, text="Upload History PDF", command=self._upload_history_pdf).pack(anchor="e", pady=5)

        self.history_preview_frame = ttk.LabelFrame(panel, text="History Preview", padding=6)
        self.history_preview_frame.pack(fill="x", pady=10)

        self.preview_image_label = ttk.Label(self.history_preview_frame, text="No preview available", anchor="center", justify="center")
        self.preview_image_label.pack(expand=True, fill="both")

        img_type_frame = ttk.LabelFrame(panel, text="Image Type", padding=12)
        img_type_frame.pack(fill="x", pady=10)
        ttk.Radiobutton(img_type_frame, text="Personal", variable=self.image_type, value="personal").pack(anchor="w")
        ttk.Radiobutton(img_type_frame, text="Generic", variable=self.image_type, value="generic").pack(anchor="w")

        ttk.Button(panel, text="Start Therapy", command=self._start_therapy_session).pack(fill="x", pady=10, ipady=10)

    def _upload_image(self):
        file_path = filedialog.askopenfilename(filetypes=[("Image Files", "*.png *.jpg *.jpeg")])
        if file_path:
            self.image_file_path = file_path
            image = Image.open(file_path)
            image.thumbnail((600, 600))
            photo = ImageTk.PhotoImage(image)
            self.image_display.configure(image=photo, text="")
            self.image_display.image = photo

            describer = ImageDescriber(image_path=file_path)
            self.image_description_widget.delete("1.0", "end")
            self.image_description_widget.insert("1.0", describer.response.output_text)

    def _upload_history_pdf(self):
        file_path = filedialog.askopenfilename(filetypes=[("PDF Files", "*.pdf")])
        if file_path:
            self.history_file_path = file_path
            self._preview_pdf(file_path)
            self.history_text = self._extract_text_from_pdf(file_path)

    def _preview_pdf(self, pdf_path):
        self.history_label.config(text=f"History: {Path(pdf_path).name}")
        doc = fitz.open(pdf_path)
        page = doc.load_page(0)
        pix = page.get_pixmap()
        mode = "RGBA" if pix.alpha else "RGB"
        img = Image.frombytes(mode, (pix.width, pix.height), pix.samples)
        img.thumbnail((500, 400))
        photo = ImageTk.PhotoImage(img)
        self.preview_image_label.configure(image=photo, text="")
        self.preview_image_label.image = photo
        doc.close()

    def _extract_text_from_pdf(self, pdf_path):
        doc = fitz.open(pdf_path)
        text = doc[0].get_text()
        doc.close()
        return text.strip()

    def _start_therapy_session(self):
        image_description = self.image_description_widget.get("1.0", "end").strip()

        if image_description and self.history_text:
            self.vapi_therapist = Vapi_TheRapist(
                image_description=image_description,
                patient_history=self.history_text
            )
            self.vapi_therapist.create_and_start_assistant()
            messagebox.showinfo("Therapy Started", "Therapy has started. Check console for details.")
        else:
            messagebox.showwarning("Missing Information", "Please upload both an image and a history PDF before starting.")


if __name__ == "__main__":
    patient_data = {
        "name": "Alex Smith",
        "age": "73"
    }

    root = tk.Tk()
    app = ReminiscenceTherapyGUI(root, patient_data)
    root.mainloop()
