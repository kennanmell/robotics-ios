//
//  GotoViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 3/4/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class GotoViewController: UIViewController, UITableViewDelegate, UITableViewDataSource {
    var gotoView: GotoView {
        return self.view as! GotoView
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        gotoView.textLabel.text = "Select a room..."
        gotoView.tableView.delegate = self
        gotoView.tableView.dataSource = self
    }
    
    // MARK: UITableViewDelegate
    
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        tableView.deselectRow(at: indexPath, animated: true)
        performSegue(withIdentifier: "GotoToPending", sender: self)
        globalNavGoal = Settings.instance.roomArray[indexPath.row]
        RequestHandler.instance.sendGoto(room: Settings.instance.roomArray[indexPath.row])
    }
    
    // MARK: UITableViewDataSource
    
    func numberOfSections(in tableView: UITableView) -> Int {
        return 1
    }
    
    func tableView(_ tableView: UITableView,
                   titleForHeaderInSection section: Int) -> String? {
        return "Rooms"
    }
    
    func tableView(_ tableView: UITableView,
                   numberOfRowsInSection section: Int) -> Int {
        return Settings.instance.roomArray.count
    }
    
    func tableView(_ tableView: UITableView,
                   cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let result = UITableViewCell()
        //result.selectionStyle = .none
        result.textLabel?.text = Settings.instance.roomArray[indexPath.row]
        result.accessoryType = .disclosureIndicator
        return result
    }
}
